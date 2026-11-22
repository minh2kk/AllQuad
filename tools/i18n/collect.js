// Lists the Vietnamese text of the configurator as shown on the demo drone (numbers as {n}) and the entries
// still missing from tools/i18n/en.json:  node tools/i18n/collect.js   (needs Playwright for Node)
const path = require("path"), fs = require("fs");
const { chromium } = require("playwright");
const PAGE = "file://" + path.join(__dirname, "..", "..", "build", "web", "AllQuad-Configurator.html");
const VI = /[àáảãạăằắẳẵặâầấẩẫậèéẻẽẹêềếểễệìíỉĩịòóỏõọôồốổỗộơờớởỡợùúủũụưừứửữựỳýỷỹỵđ]/i;
const grab = (pg, all) => pg.evaluate((src) => { const VI = new RegExp(src, "i"); const out = []; const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  let n; while ((n = w.nextNode())) { if (n.parentElement && /SCRIPT|STYLE/.test(n.parentElement.tagName)) continue; const v = n.nodeValue.trim(); if (v && VI.test(v)) out.push(v); }
  document.querySelectorAll("[title],[placeholder],[aria-label]").forEach((e) => ["title","placeholder","aria-label"].forEach((a) => { const v = e.getAttribute(a); if (v && VI.test(v)) out.push(v.trim()); }));
  document.querySelectorAll("option").forEach((o) => { if (VI.test(o.textContent)) out.push(o.textContent.trim()); });
  return out; }, VI.source).then((s) => s.forEach((x) => all.add(x)));
(async () => {
  const b = await chromium.launch(); const pg = await b.newPage({ viewport: { width: 1366, height: 900 } });
  const all = new Set();
  await pg.goto(PAGE);
  await pg.waitForTimeout(500); await grab(pg, all);
  await pg.click("#connect .flOpen"); await pg.waitForTimeout(300); await grab(pg, all); await pg.click("#flClose");
  await pg.click("#demoBtn"); await pg.waitForFunction("Object.keys(S.params).length > 200 && S.status", null, { timeout: 20000 });
  await pg.waitForTimeout(1500);
  const tabs = await pg.evaluate("[...document.querySelectorAll('nav.tabs button[data-tab]')].map(b=>b.dataset.tab)");
  for (const t of tabs) { await pg.evaluate((t) => gotoTab(t), t); await pg.waitForTimeout(800); await grab(pg, all); }
  await pg.evaluate("searchOpen(); $('gsIn').value=''; searchRun()"); await pg.waitForTimeout(200); await grab(pg, all);
  await pg.evaluate("$('searchDlg').hidden=true; glossShow()"); await pg.waitForTimeout(200); await grab(pg, all);
  await pg.evaluate("$('glossDlg').hidden=true; gotoTab('preflight'); ffLive('orient')"); await pg.waitForTimeout(300); await grab(pg, all);
  await pg.evaluate("ffLiveStop(); ffLive('fs')"); await pg.waitForTimeout(300); await grab(pg, all); await pg.evaluate("ffLiveStop()");
  // strings kept in JS tables that may not be on screen now
  const js = await pg.evaluate((src) => { const VI = new RegExp(src, "i"); const out = [];
    const add = (v) => { if (typeof v === "string" && VI.test(v)) out.push(v.trim()); };
    Object.values(HINT).forEach((h) => { add(h); if (typeof splitHint === "function") splitHint(h).forEach(add); add(h.split(":")[0]); });
    Object.values(ENUMS).forEach((e) => Array.isArray(e) ? e.forEach(add) : 0);
    Object.values(BITS).forEach((e) => e.forEach(([, t]) => add(t)));
    Object.values(BLOCK_HELP).forEach((e) => e.forEach(add));
    FF_STEPS.forEach((s) => { add(s.t); add(s.why); s.act.forEach(([l]) => add(l)); });
    Object.values(GLOSS).forEach(add); Object.values(EFFECT).forEach((e) => e.forEach((x) => add("↑ " + x) || add("↓ " + x)));
    SEARCH_ACTIONS.forEach(([t]) => add(t));
    return out; }, VI.source);
  js.forEach((x) => all.add(x));
  const num = /(?<![\w.])-?\d+(?:[.,]\d+)?/g;
  const tpl = [...new Set([...all].filter((x) => x.length < 600).map((x) => x.replace(num, "{n}")))];
  const have = JSON.parse(fs.readFileSync(path.join(__dirname, "en.json"), "utf8"));
  const rule = /^(Giảm|Tăng) \S+$|^Giải thích: |^[↑↓] /;
  const missing = tpl.filter((x) => !(x in have) && !rule.test(x)).sort();
  fs.writeFileSync(path.join(__dirname, "missing.json"), JSON.stringify(missing, null, 1));
  console.log(`${tpl.length} strings on screen, ${missing.length} without English (tools/i18n/missing.json)`);
  await b.close();
})();
