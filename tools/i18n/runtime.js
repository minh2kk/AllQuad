// AllQuad configurator - English view (generated: tools/i18n/make_i18n.py from tools/i18n/en.json)
// The page is written in Vietnamese. With English on, every Vietnamese text node and title / placeholder /
// aria-label is looked up in the table below, numbers swapped for {n} (so "12 vệ tinh" and "8 vệ tinh" share
// one entry). Text without an entry stays Vietnamese. Switching back restores the original text.
"use strict";
const I18N = { on: false, orig: new WeakMap(), cache: new Map(), obs: null, dict: null, plain: new Set() };
const I18N_VI = /[àáảãạăằắẳẵặâầấẩẫậèéẻẽẹêềếểễệìíỉĩịòóỏõọôồốổỗộơờớởỡợùúủũụưừứửữựỳýỷỹỵđ]/i;
const I18N_NUM = /(?<![\w.])-?\d+(?:[.,]\d+)?/g;
const I18N_RULES = [
  [/^Giảm (\S+)$/, (m) => "Decrease " + m[1]],
  [/^Tăng (\S+)$/, (m) => "Increase " + m[1]],
  [/^Giải thích: (.+)$/, (m) => "Explain: " + m[1]],
  [/^([↑↓]) (.+)$/, (m) => { const t = i18nText(m[2]); return t ? m[1] + " " + t : null; }],
];
function i18nText(s) {
  if (I18N.cache.has(s)) return I18N.cache.get(s);
  const nums = [];
  const key = s.replace(I18N_NUM, (x) => (nums.push(x), "{n}"));
  // Vietnamese without diacritics ("Pin", "Bay"): only the exact entries of the table
  if (!I18N_VI.test(s) && !I18N.plain.has(key)) return null;
  let en = I18N.dict.get(key);
  if (en !== undefined) { let k = 0; en = en.replace(/\{n\}/g, () => (k < nums.length ? nums[k++] : "")); }
  else {
    en = null;
    for (const [re, fn] of I18N_RULES) { const m = re.exec(s); if (m) { en = fn(m); break; } }
  }
  if (I18N.cache.size > 4000) I18N.cache.clear();
  I18N.cache.set(s, en);
  return en;
}
function i18nNode(n) {
  if (n.nodeType === 3) {
    const v = n.nodeValue, t = v.trim();
    if (!t || !(I18N_VI.test(t) || I18N.plain.has(t.replace(I18N_NUM, "{n}")))) return;
    const p = n.parentElement;
    if (p && (p.tagName === "SCRIPT" || p.tagName === "STYLE" || p.closest("textarea, .cli-out, #cliOut, [data-noi18n]"))) return;
    const en = i18nText(t);
    if (!en) return;
    I18N.orig.set(n, v);
    n.nodeValue = v.replace(t, en);
  } else if (n.nodeType === 1) {
    for (const a of ["title", "placeholder", "aria-label"]) {
      const v = n.getAttribute(a);
      if (!v || !I18N_VI.test(v)) continue;
      const en = i18nText(v.trim());
      if (!en) continue;
      const o = I18N.orig.get(n) || {};
      o[a] = v; I18N.orig.set(n, o);
      n.setAttribute(a, en);
    }
  }
}
function i18nTree(root) {
  i18nNode(root);
  const w = document.createTreeWalker(root, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
  let n;
  while ((n = w.nextNode())) i18nNode(n);
}
function i18nRestore() {
  const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
  let n;
  while ((n = w.nextNode())) {
    const o = I18N.orig.get(n);
    if (o === undefined) continue;
    if (n.nodeType === 3) n.nodeValue = o;
    else for (const a in o) n.setAttribute(a, o[a]);
    I18N.orig.delete(n);
  }
}
function i18nSet(on) {
  I18N.on = on;
  try { localStorage.setItem("aq-lang", on ? "en" : "vi"); } catch (e) {}
  document.documentElement.lang = on ? "en" : "vi";
  const b = document.getElementById("langBtn");
  if (b) { b.querySelector("span").textContent = on ? "EN" : "VI"; b.setAttribute("aria-pressed", String(on)); }
  document.querySelectorAll(".lang-sw").forEach((x) => (x.textContent = on ? "Tiếng Việt" : "English"));
  if (on) {
    if (!I18N.dict) {
      I18N.dict = new Map(Object.entries(I18N_EN));
      I18N.plain = new Set([...I18N.dict.keys()].filter((k) => !I18N_VI.test(k)));
    }
    i18nTree(document.body);
    if (!I18N.obs) {
      I18N.obs = new MutationObserver((ms) => {
        if (!I18N.on) return;
        for (const m of ms) {
          if (m.type === "characterData") i18nNode(m.target);
          else if (m.type === "attributes") i18nNode(m.target);
          else m.addedNodes.forEach((x) => (x.nodeType === 1 ? i18nTree(x) : i18nNode(x)));
        }
      });
      I18N.obs.observe(document.body, { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ["title", "placeholder", "aria-label"] });
    }
  } else i18nRestore();
}
{
  const hdr = document.getElementById("themeQuick");
  if (hdr) {
    const b = document.createElement("button");
    b.type = "button"; b.className = "st-btn"; b.id = "langBtn";
    b.title = "Ngôn ngữ / Language";
    b.innerHTML = '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M3 12h18M12 3a14 14 0 0 1 0 18M12 3a14 14 0 0 0 0 18"/></svg><span>VI</span>';
    b.onclick = () => i18nSet(!I18N.on);
    hdr.after(b);
  }
  // the connect / login screens cover the header: a switch of their own (always shows the other language)
  for (const id of ["connect", "login"]) {
    const d = document.querySelector("#" + id + " > .panel");
    if (!d) continue;
    const s = document.createElement("button");
    s.type = "button"; s.className = "linkbtn lang-sw"; s.dataset.noi18n = "1"; s.textContent = "English";
    s.style.cssText = "float:right;font-size:.85rem";
    s.onclick = (e) => { e.preventDefault(); i18nSet(!I18N.on); };
    d.prepend(s);
  }
  let saved = "vi";
  try { saved = localStorage.getItem("aq-lang") || "vi"; } catch (e) {}
  if (saved === "en") i18nSet(true);
}
