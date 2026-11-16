-- AllQuad telemetry screen for EdgeTX black & white radios (128x64: RadioMaster Pocket, Zorro, Boxer,
-- TX12, Jumper T-Lite ...). Reads the CRSF sensors the flight controller sends through ExpressLRS:
--   RxBt Curr Capa Bat%  FM  VSpd  Alt  GPS GSpd Hdg Sats  Ptch Roll Yaw  RQly 1RSS 2RSS RSNR TPWR
-- Pages (ENTER / roller press or PAGE> on the Pocket):  1 flight  2 link & GPS  3 last-flight summary
-- Install: copy to /SCRIPTS/TELEMETRY/aqtlm.lua on the SD card, then MODEL -> DISPLAY -> Screen 1:
-- Script, aqtlm. Long-press TELE (or PAGE) on the main screen to open it.

local CELL_WARN = 3.5      -- V per cell: blinking battery + one short vibration
local CELL_CRIT = 3.3      -- V per cell: three vibrations
local LQ_WARN = 50         -- link quality % to blink

local page = 1
local cells = 0
local armed, armT, flyT = false, 0, 0
local home = nil
local warnLvl = 0
local mx = { alt = 0, spd = 0, dist = 0, vmin = 99, amax = 0, mah = 0, t = 0, has = false }
local ids = {}

-- sensor id lookup once (getValue by id is faster than by name, and a missing sensor stays nil)
local function sid(name)
  if ids[name] == nil then
    local f = getFieldInfo(name)
    ids[name] = f and f.id or false
  end
  return ids[name]
end
local function val(name, def)
  local id = sid(name)
  if not id then return def end
  local v = getValue(id)
  if v == nil then return def end
  return v
end

-- altitude: barometric "Alt" first, the GPS altitude otherwise
local function altVal()
  if sid("Alt") then return val("Alt", 0) end
  return val("GAlt", 0)
end
local function dist(a, b)  -- metres between two {lat, lon}
  local r = 6371000
  local dLat = math.rad(b.lat - a.lat)
  local dLon = math.rad(b.lon - a.lon)
  local x = dLon * math.cos(math.rad((a.lat + b.lat) / 2))
  return r * math.sqrt(x * x + dLat * dLat)
end
local function bearing(a, b)  -- degrees from a to b
  local y = math.sin(math.rad(b.lon - a.lon)) * math.cos(math.rad(b.lat))
  local x = math.cos(math.rad(a.lat)) * math.sin(math.rad(b.lat)) - math.sin(math.rad(a.lat)) * math.cos(math.rad(b.lat)) * math.cos(math.rad(b.lon - a.lon))
  return (math.deg(math.atan2(y, x)) + 360) % 360
end
local function mmss(s)
  s = math.floor(s)
  return string.format("%02d:%02d", math.floor(s / 60), s % 60)
end

-- small arrow pointing to `deg` (0 = up) centred on cx, cy
local function arrow(cx, cy, deg, r)
  local a = math.rad(deg)
  local sx, sy = math.sin(a), -math.cos(a)
  local tx, ty = cx + sx * r, cy + sy * r
  local bx, by = cx - sx * r * 0.6, cy - sy * r * 0.6
  lcd.drawLine(bx, by, tx, ty, SOLID, FORCE)
  local l = math.rad(deg + 150)
  local rr = math.rad(deg - 150)
  lcd.drawLine(tx, ty, tx + math.sin(l) * r * 0.7, ty - math.cos(l) * r * 0.7, SOLID, FORCE)
  lcd.drawLine(tx, ty, tx + math.sin(rr) * r * 0.7, ty - math.cos(rr) * r * 0.7, SOLID, FORCE)
end

local function init()
  page = 1
end

-- everything that must keep running while another screen is shown
local function background()
  local fm = val("FM", "")
  if type(fm) ~= "string" then fm = "" end
  local nowArmed = fm ~= "" and string.sub(fm, -1) ~= "*" and fm ~= "!FS!"
  local now = getTime() / 100
  local v = val("RxBt", 0)
  -- cell count from the first real reading, before take-off
  if v > 3 and (cells == 0 or not nowArmed) then cells = math.max(1, math.floor(v / 4.35) + 1) end
  local gps = val("GPS", 0)
  local sats = val("Sats", 0)
  if nowArmed and not armed then
    armT = now
    mx = { alt = 0, spd = 0, dist = 0, vmin = 99, amax = 0, mah = 0, t = 0, has = true }
    home = (type(gps) == "table" and sats >= 6) and { lat = gps.lat, lon = gps.lon } or nil
  end
  if nowArmed then
    flyT = now - armT
    mx.t = flyT
    mx.alt = math.max(mx.alt, altVal())
    mx.spd = math.max(mx.spd, val("GSpd", 0))
    mx.amax = math.max(mx.amax, val("Curr", 0))
    if v > 1 then mx.vmin = math.min(mx.vmin, v) end
    mx.mah = val("Capa", 0)
    if not home and type(gps) == "table" and sats >= 6 then home = { lat = gps.lat, lon = gps.lon } end
    if home and type(gps) == "table" then mx.dist = math.max(mx.dist, dist(home, gps)) end
  end
  if armed and not nowArmed and mx.has then page = 3 end  -- landed: show the summary
  armed = nowArmed
  -- vibration once per new warning level
  local cv = (cells > 0 and v > 1) and v / cells or 0
  local lvl = (cv > 0 and cv < CELL_CRIT) and 2 or ((cv > 0 and cv < CELL_WARN) and 1 or 0)
  if lvl > warnLvl and armed then
    playHaptic(15, 100)
    if lvl == 2 then playHaptic(15, 100); playHaptic(15, 100) end
  end
  if lvl == 0 or not armed then warnLvl = 0 elseif lvl > warnLvl then warnLvl = lvl end
end

local function topBar(fm, lq)
  lcd.drawFilledRectangle(0, 0, LCD_W, 9, FORCE)
  local m = fm ~= "" and fm or "NO TELEM"
  lcd.drawText(1, 1, m, INVERS)
  if armed then lcd.drawText(LCD_W / 2 - 14, 1, mmss(flyT), INVERS) end
  lcd.drawText(LCD_W - 1, 1, "LQ" .. lq, INVERS + RIGHT + ((lq < LQ_WARN) and BLINK or 0))
end

local function pageFlight()
  local fm = val("FM", "")
  if type(fm) ~= "string" then fm = "" end
  local lq = val("RQly", 0)
  topBar(fm, lq)
  if fm == "" and val("RxBt", 0) == 0 then  -- nothing from the drone yet
    lcd.drawText(4, 18, "NO TELEMETRY", MIDSIZE + BLINK)
    lcd.drawText(2, 36, "Drone on? RX bound?", SMLSIZE)
    lcd.drawText(2, 44, "ELRS: telemetry ratio on", SMLSIZE)
    lcd.drawText(2, 52, "MODEL>TELEMETRY>Discover", SMLSIZE)
    return
  end
  -- battery block (left)
  local v = val("RxBt", 0)
  local cv = (cells > 0 and v > 1) and v / cells or 0
  local low = cv > 0 and cv < CELL_WARN
  lcd.drawNumber(1, 12, math.floor(v * 10 + 0.5), DBLSIZE + PREC1 + (low and BLINK or 0))
  lcd.drawText(lcd.getLastRightPos(), 19, "V", 0)
  local pct = val("Bat%", 0)
  lcd.drawText(1, 30, string.format("%.2fV/%dS %d%%", cv, cells, pct), SMLSIZE)
  lcd.drawRectangle(1, 38, 62, 7, SOLID)
  local w = math.floor(60 * math.min(100, math.max(0, pct)) / 100)
  if w > 0 then lcd.drawFilledRectangle(2, 39, w, 5, 0) end
  -- flight values (right)
  local x = 70
  lcd.drawLine(66, 11, 66, 45, DOTTED, FORCE)
  local alt, vs, spd, sats = altVal(), val("VSpd", 0), val("GSpd", 0), val("Sats", 0)
  lcd.drawText(x, 12, "ALT", SMLSIZE)
  lcd.drawText(LCD_W - 1, 11, string.format("%.1fm", alt), RIGHT)
  lcd.drawText(x, 21, "VSP", SMLSIZE)
  lcd.drawText(LCD_W - 1, 20, string.format("%+.1f", vs), RIGHT)
  lcd.drawText(x, 30, "SPD", SMLSIZE)
  lcd.drawText(LCD_W - 1, 29, string.format("%dkm/h", math.floor(spd + 0.5)), RIGHT)
  lcd.drawText(x, 39, "SAT", SMLSIZE)
  lcd.drawText(LCD_W - 1, 38, tostring(sats), RIGHT + ((sats > 0 and sats < 6) and BLINK or 0))
  -- bottom: home, current, mAh
  lcd.drawLine(0, 47, LCD_W - 1, 47, SOLID, FORCE)
  local gps = val("GPS", 0)
  if home and type(gps) == "table" then
    local d = dist(home, gps)
    local brg = bearing(gps, home)
    local hdg = val("Hdg", 0)
    lcd.drawText(1, 50, "HOME", SMLSIZE)
    lcd.drawText(23, 49, d >= 1000 and string.format("%.2fkm", d / 1000) or string.format("%dm", math.floor(d + 0.5)), 0)
    arrow(58, 55, (brg - hdg + 360) % 360, 5)
  else
    lcd.drawText(1, 50, "HOME --", SMLSIZE)
  end
  lcd.drawText(LCD_W - 1, 49, string.format("%.1fA", val("Curr", 0)), RIGHT)
  lcd.drawText(LCD_W - 1, 57, val("Capa", 0) .. "mAh", SMLSIZE + RIGHT)
  lcd.drawText(1, 58, "TX " .. val("TPWR", 0) .. "mW", SMLSIZE)
end

local function pageLink()
  local fm = val("FM", "")
  if type(fm) ~= "string" then fm = "" end
  topBar(fm, val("RQly", 0))
  -- mini horizon (left)
  local cx, cy, r = 24, 36, 22
  lcd.drawRectangle(cx - r, cy - r + 2, r * 2, r * 2 - 4, SOLID)
  local roll, pitch = val("Roll", 0), val("Ptch", 0)
  -- the sensors are in radians on most EdgeTX versions, degrees on some: normalise to degrees
  if math.abs(roll) < 3.2 and math.abs(pitch) < 3.2 then roll, pitch = math.deg(roll), math.deg(pitch) end
  local a = math.rad(roll)
  local off = math.max(-r + 4, math.min(r - 4, pitch * 0.6))
  local dx, dy = math.cos(a) * (r - 3), math.sin(a) * (r - 3)
  lcd.drawLine(cx - dx, cy + off + dy, cx + dx, cy + off - dy, SOLID, FORCE)
  lcd.drawLine(cx - 6, cy, cx - 2, cy, SOLID, FORCE)
  lcd.drawLine(cx + 2, cy, cx + 6, cy, SOLID, FORCE)
  lcd.drawPoint(cx, cy)
  -- link and GPS (right)
  local x = 52
  lcd.drawText(x, 12, string.format("RSSI %d/%d", val("1RSS", 0), val("2RSS", 0)), SMLSIZE)
  lcd.drawText(x, 20, string.format("SNR %d  PWR %d", val("RSNR", 0), val("TPWR", 0)), SMLSIZE)
  lcd.drawText(x, 28, string.format("R %.0f P %.0f", roll, pitch), SMLSIZE)
  local yaw = val("Yaw", 0)
  if math.abs(yaw) < 6.3 then yaw = math.deg(yaw) end
  lcd.drawText(x, 36, string.format("HDG %d", math.floor((val("Hdg", yaw) + 360) % 360)), SMLSIZE)
  local gps = val("GPS", 0)
  if type(gps) == "table" then
    lcd.drawText(x, 46, string.format("%.5f", gps.lat), SMLSIZE)
    lcd.drawText(x, 54, string.format("%.5f", gps.lon), SMLSIZE)
  else
    lcd.drawText(x, 46, "GPS: no fix", SMLSIZE)
  end
end

local function pageSummary()
  lcd.drawFilledRectangle(0, 0, LCD_W, 9, FORCE)
  lcd.drawText(1, 1, "LAST FLIGHT", INVERS)
  lcd.drawText(LCD_W - 1, 1, mx.has and mmss(mx.t) or "--:--", INVERS + RIGHT)
  if not mx.has then
    lcd.drawText(4, 28, "No flight yet", 0)
    return
  end
  local rows = {
    { "Max alt", string.format("%.1f m", mx.alt) },
    { "Max speed", string.format("%d km/h", math.floor(mx.spd + 0.5)) },
    { "Max dist", string.format("%d m", math.floor(mx.dist + 0.5)) },
    { "Min volt", mx.vmin < 99 and string.format("%.1fV %.2f/c", mx.vmin, cells > 0 and mx.vmin / cells or 0) or "--" },
    { "Max curr", string.format("%.1f A", mx.amax) },
    { "Used", mx.mah .. " mAh" },
  }
  for i, r in ipairs(rows) do
    local y = 3 + i * 9
    lcd.drawText(2, y, r[1], SMLSIZE)
    lcd.drawText(LCD_W - 1, y - 1, r[2], RIGHT)
  end
end

local function run(event)
  background()
  if event == EVT_VIRTUAL_ENTER or event == EVT_VIRTUAL_NEXT then page = page % 3 + 1
  elseif event == EVT_VIRTUAL_PREV then page = (page + 1) % 3 + 1 end
  lcd.clear()
  if page == 1 then pageFlight() elseif page == 2 then pageLink() else pageSummary() end
  return 0
end

return { init = init, background = background, run = run }
