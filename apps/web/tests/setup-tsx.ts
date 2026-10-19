import { registerHooks } from 'node:module';
import { readFileSync } from 'node:fs';
import ts from 'typescript';
export const hooks = registerHooks({
  load(url, context, next) {
    if (url.endsWith('.tsx')) return { format: 'module', shortCircuit: true, source: ts.transpileModule(readFileSync(new URL(url), 'utf8'), {
      fileName: new URL(url).pathname, compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2022 } }).outputText };
    return next(url, context);
  },
});
