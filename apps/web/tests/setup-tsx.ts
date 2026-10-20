import { registerHooks } from 'node:module';
import { readFileSync } from 'node:fs';
import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { Window } from 'happy-dom';
import ts from 'typescript';

export const hooks = registerHooks({
  load(url, context, next) {
    if (url.endsWith('.tsx')) return { format: 'module', shortCircuit: true, source: ts.transpileModule(readFileSync(new URL(url), 'utf8'), {
      fileName: new URL(url).pathname, compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2022 } }).outputText };
    return next(url, context);
  },
});

export function dom(url = 'http://127.0.0.1:57000') {
  const window = new Window({ url });
  Object.assign(globalThis, { window, document: window.document, HTMLElement: window.HTMLElement, IS_REACT_ACT_ENVIRONMENT: true });
  const element = window.document.createElement('div');
  window.document.body.append(element);
  return { window, element, root: createRoot(element as unknown as Element) };
}

export async function flush(ms = 10) {
  await act(async () => { await new Promise(resolve => setTimeout(resolve, ms)); });
}
