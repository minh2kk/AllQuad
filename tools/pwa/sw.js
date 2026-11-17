// AllQuad configurator: service worker of the GitHub Pages site (copied there by tools/make_pages.py).
// The app opens without internet (last copy from the cache) and is never stale when online:
// every page / manifest / firmware request goes to the network first and refreshes the cache;
// the cache answers only when the network does not (offline, or no answer within a few seconds).
// version.json is never cached: the page polls it to see when a newer build was published.
const CACHE = "allquad-pages-v1";
const CORE = ["./", "manifest.webmanifest", "icon-192.png", "icon-512.png", "firmware/manifest.json"];
const NET_WAIT_MS = 4000;

self.addEventListener("install", (e) => {
  // a missing optional file must not stop the install
  e.waitUntil(caches.open(CACHE).then((c) => Promise.all(CORE.map((u) => c.add(new Request(u, { cache: "reload" })).catch(() => {}))))
    .then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(caches.keys().then((ks) => Promise.all(ks.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener("message", (e) => { if (e.data === "skipWaiting") self.skipWaiting(); });

function fromNetwork(req) {
  return new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error("slow")), NET_WAIT_MS);
    fetch(req, { cache: "no-store" }).then((r) => {
      clearTimeout(t);
      if (r.ok && r.type === "basic") {
        // the page itself is kept under one key ("./"), whatever query it was loaded with (?v=<build>
        // after an update): offline, the latest page opened is the one that comes back
        const page = req.mode === "navigate" || /\/(index\.html)?$/.test(new URL(req.url).pathname);
        const copy = r.clone();
        caches.open(CACHE).then((c) => c.put(page ? new Request(self.registration.scope) : req, copy)).catch(() => {});
      }
      resolve(r);
    }, (err) => { clearTimeout(t); reject(err); });
  });
}

self.addEventListener("fetch", (e) => {
  const req = e.request;
  const u = new URL(req.url);
  if (req.method !== "GET" || u.origin !== self.location.origin) return;  // FC over WiFi, GitHub API: untouched
  if (u.pathname.endsWith("/version.json")) return;                       // always live
  const page = req.mode === "navigate" || /\/(index\.html)?$/.test(u.pathname);
  e.respondWith(fromNetwork(req).catch(() => (page ? caches.match(self.registration.scope) : caches.match(req, { ignoreSearch: true }))
    .then((hit) => hit || caches.match(req, { ignoreSearch: true }))
    .then((hit) => hit || fetch(req))));
});
