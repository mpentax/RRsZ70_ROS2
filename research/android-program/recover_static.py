#!/usr/bin/env python3
"""Recover exact static literals, NEVER generate programs or execute vendor code.
Only allowlisted disassembly windows are exported. No SDK, network or subprocess.
"""
import argparse, ast, hashlib, json, os, re
from pathlib import Path

WINDOWS = {'asset-registration': (1429910,1429930), 'webview-source': (1425290,1425320),
           'blockly-consumer': (1425426,1425498), 'm4-preset-wire': (1430378,1430390),
           'm4-preset-block': (1430424,1430434), 'asset-resolver': (96903,97039)}

def literal(text):
    # Token conversion outside quoted strings only; literal_eval never runs code.
    tokens = re.split(r"('(?:[^'\\]|\\.)*'|\"(?:[^\"\\]|\\.)*\")", text)
    for i in range(0,len(tokens),2):
        tokens[i] = re.sub(r'\b(null|true|false)\b', lambda m: {'null':'None','true':'True','false':'False'}[m[0]], tokens[i])
    return ast.literal_eval(''.join(tokens))

def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def recover(hasm):
    lines = Path(hasm).read_text().splitlines()
    records=[]
    # These are fixed primary preset windows, not a search-and-invent generator.
    for start,end in [(1430378,1430390),(1430424,1430434)]:
        region=lines[start-1:end]
        for j,s in enumerate(region):
            if '# Object: ' not in s: continue
            obj=literal(s.split('# Object: ',1)[1]); number=start+j
            if obj.get('cmd_id')=='arm_joints' and obj.get('value') is None:
                nxt=region[j+1:j+3]
                if len(nxt)!=2 or '# Array: ' not in nxt[0] or "# String: 'value'" not in nxt[1]:
                    raise ValueError('Primary register-assignment pattern changed')
                regs=re.findall(r'Reg8: (\d+)',s.split('# Object:')[0])
                arrregs=re.findall(r'Reg8: (\d+)',nxt[0].split('# Array:')[0])
                putregs=re.findall(r'Reg8: (\d+)',nxt[1].split('# String:')[0])
                if putregs[:2] != [regs[0],arrregs[0]]: raise ValueError('Register mismatch')
                obj['value']=literal(nxt[0].split('# Array: ',1)[1])
                records.append({'kind':'static-preset-wire-record','lines':[number,number+2],'record':obj})
            elif obj.get('type')=='arm_m4':
                fieldline=region[j+1]; putline=region[j+2]
                if '# Object: ' not in fieldline or "# String: 'fields'" not in putline:raise ValueError('Field assignment missing')
                obj['fields']=literal(fieldline.split('# Object: ',1)[1])
                records.append({'kind':'static-preset-block-node','lines':[number,number+2],'record':obj})
    return {'classification':'static-original-preset-literals-not-UI-generated',
            'hasm_sha256':digest(hasm),'records':records,
            'requested_M4_up_step1_generated':False,'original_generator_recovered':False}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--hasm',required=True);ap.add_argument('--out',required=True);a=ap.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True,mode=0o700);os.chmod(out,0o700)
    evidence=recover(a.hasm)
    lines=Path(a.hasm).read_text().splitlines()
    for name,(s,e) in WINDOWS.items():
        p=out/(name+'.hasm.txt');p.write_text('\n'.join(f'{i}: {lines[i-1]}' for i in range(s,e+1))+'\n');os.chmod(p,0o600)
    evidence['window_sha256']={p.name:digest(p) for p in sorted(out.glob('*.hasm.txt'))}
    p=out/'static-m4-evidence.json';p.write_text(json.dumps(evidence,indent=2)+'\n');os.chmod(p,0o600)
    print(json.dumps({'classification':evidence['classification'],'records':len(evidence['records']),'out':str(out)}))

if __name__=='__main__':main()
