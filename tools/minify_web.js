// Minifies the page's <script> / <style> blocks with terser + csso (npm install in the project, or
// NODE_PATH pointing at them). Called by tools/embed_web.py; stdin / stdout: JSON {js: [...], css: [...]}.
// Top-level names are kept (the scripts share globals and the tests call them), locals are mangled.
const { minify } = require("terser");
const csso = require("csso");
let raw = "";
process.stdin.on("data", (d) => (raw += d));
process.stdin.on("end", async () => {
  const { js, css } = JSON.parse(raw);
  const out = { js: [], css: [] };
  for (const code of js) {
    const r = await minify(code, { ecma: 2020, toplevel: false, compress: { passes: 2, keep_fargs: true }, mangle: true, format: { comments: false } });
    out.js.push(r.code);
  }
  for (const c of css) out.css.push(csso.minify(c).css);
  process.stdout.write(JSON.stringify(out));
});
