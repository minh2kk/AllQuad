-- AllQuad settings from the radio (EdgeTX, black & white 128x64: RadioMaster Pocket, Zorro, Boxer, TX12 ...)
-- PID, rates, filters and VTX of the flight controller, read and written over the ExpressLRS / Crossfire link
-- (MSP inside CRSF frames: the FC must use CRSF on its receiver port). Disarmed only.
-- Install: copy to /SCRIPTS/TOOLS/aqcfg.lua on the SD card; SYS -> Tools -> aqcfg.
-- Keys: roller / +- = move, ENTER = edit (roller changes, ENTER sends it, EXIT cancels),
--       PAGE> / <PAGE (or long ENTER) = next page, "SAVE TO FC" item = keep after power-off.

local FC, RADIO = 0xC8, 0xEA
local REQ, RESP, WRITE = 0x7A, 0x7B, 0x7C
local CMD_FIND, CMD_SET, CMD_SAVE = 0x4152, 0x4153, 250

-- name shown, parameter, step, decimals
local PAGES = {
  { title = "PID", items = {
    { "Roll P", "roll_p", 1, 0 }, { "Roll I", "roll_i", 1, 0 }, { "Roll D", "roll_d", 1, 0 }, { "Roll FF", "roll_f", 5, 0 },
    { "Pitch P", "pitch_p", 1, 0 }, { "Pitch I", "pitch_i", 1, 0 }, { "Pitch D", "pitch_d", 1, 0 }, { "Pitch FF", "pitch_f", 5, 0 },
    { "Yaw P", "yaw_p", 1, 0 }, { "Yaw I", "yaw_i", 1, 0 }, { "Yaw FF", "yaw_f", 5, 0 },
    { "TPA", "tpa_rate", 0.05, 2 }, { "Anti grav", "anti_gravity", 0.5, 1 } } },
  { title = "RATES", items = {
    { "Roll ctr", "roll_rc_rate", 5, 0 }, { "Roll max", "roll_max_rate", 10, 0 }, { "Roll expo", "roll_expo", 0.05, 2 },
    { "Pitch ctr", "pitch_rc_rate", 5, 0 }, { "Pitch max", "pitch_max_rate", 10, 0 }, { "Pitch expo", "pitch_expo", 0.05, 2 },
    { "Yaw ctr", "yaw_rc_rate", 5, 0 }, { "Yaw max", "yaw_max_rate", 10, 0 }, { "Yaw expo", "yaw_expo", 0.05, 2 } } },
  { title = "FILTER", items = {
    { "Gyro LPF", "gyro_lpf_hz", 5, 0 }, { "Gyro LPF2", "gyro_lpf2_hz", 5, 0 }, { "Dterm LPF", "dterm_lpf_hz", 5, 0 },
    { "Notches", "dyn_notch_count", 1, 0 }, { "Notch min", "dyn_notch_min", 5, 0 }, { "Notch max", "dyn_notch_max", 10, 0 } } },
  { title = "VTX", items = {
    { "Band", "vtx_band", 1, 0 }, { "Channel", "vtx_channel", 1, 0 }, { "Power", "vtx_power", 1, 0 }, { "Pit mode", "vtx_pit", 1, 0 } } },
}
local BANDS = { "A", "B", "E", "F", "R", "L", "U" }

local page, sel, top = 1, 1, 1
local editing, editVal = false, 0
local params = {}            -- name -> { idx, v, min, max }  (values x1000)
local queue = {}             -- requests waiting: { cmd, data, name?, kind }
local busy = nil             -- request in flight { ..., sent, tries }
local status, statusT = "", 0
local seq = 0
local rx = nil               -- answer being assembled

-- ---------------------------------------------------------------- MSP over CRSF
local function u16(t, v) t[#t + 1] = v % 256; t[#t + 1] = math.floor(v / 256) % 256 end
local function i32(t, v)
  if v < 0 then v = v + 4294967296 end
  for _ = 1, 4 do t[#t + 1] = v % 256; v = math.floor(v / 256) end
end
local function rd32(d, i)
  local v = d[i] + d[i + 1] * 256 + d[i + 2] * 65536 + d[i + 3] * 16777216
  if v >= 2147483648 then v = v - 4294967296 end
  return v
end

local function sendChunks(cmd, data)
  local body = {}
  local v2 = cmd > 255
  if v2 then
    body = { 0 }; u16(body, cmd); u16(body, #data)
  else
    body = { #data, cmd }
  end
  local x = 0
  for i = 1, #data do body[#body + 1] = data[i] end
  if not v2 then
    for i = 1, #body do x = bit32.bxor(x, body[i]) end
    body[#body + 1] = x
  end
  local pos = 1
  local first = true
  while pos <= #body do
    local st = (v2 and 0x40 or 0x20) + (first and 0x10 or 0) + seq
    seq = (seq + 1) % 16
    local p = { FC, RADIO, st }
    for i = pos, math.min(pos + 6, #body) do p[#p + 1] = body[i] end
    pos = pos + 7
    first = false
    crossfireTelemetryPush(REQ, p)
  end
end

-- one 0x7B frame: returns true, err, payload when an answer is complete
local function onChunk(d)
  if d[1] ~= RADIO or d[2] ~= FC or #d < 3 then return false end
  local st = d[3]
  local err = st >= 128
  local ver = math.floor(st / 32) % 4
  local start = math.floor(st / 16) % 2 == 1
  local sq = st % 16
  local i = 4
  if start then
    rx = { err = err, data = {}, seq = sq }
    if ver == 2 then
      rx.size = d[i + 3] + d[i + 4] * 256; i = i + 5
    else
      rx.size = d[i]; i = i + 2
    end
  else
    if not rx or sq ~= (rx.seq + 1) % 16 then rx = nil; return false end
    rx.seq = sq
  end
  while i <= #d and #rx.data < rx.size do rx.data[#rx.data + 1] = d[i]; i = i + 1 end
  if #rx.data < rx.size then return false end
  local r = rx
  rx = nil
  return true, r.err, r.data
end

local function setStatus(s) status = s; statusT = getTime() end

-- false: the answer belongs to an older request (sent again after a timeout): keep waiting
local function onAnswer(err, d)
  local q = busy
  if not err and q.kind ~= "save" and #d >= 18 then
    local name = ""
    for i = 19, #d do name = name .. string.char(d[i]) end
    if name ~= q.name then return false end
  end
  busy = nil
  if q.kind == "save" then
    setStatus((not err and d[1] == 1) and "Saved" or "Save failed (armed?)")
    return true
  end
  if err then
    if q.kind == "set" then setStatus("Refused (armed?)") end
    if q.kind == "find" then params[q.name] = { missing = true } end
    return true
  end
  if #d < 18 then return true end
  local name = ""
  for i = 19, #d do name = name .. string.char(d[i]) end
  params[name] = { idx = d[1] + d[2] * 256, v = rd32(d, 7), min = rd32(d, 11), max = rd32(d, 15) }
  if q.kind == "set" then setStatus("Set " .. name) end
  return true
end

local function pump()
  -- answers
  for _ = 1, 8 do
    local cmd, data = crossfireTelemetryPop()
    if cmd == nil then break end
    if cmd == RESP and busy then
      local done, err, payload = onChunk(data)
      if done then onAnswer(err, payload) end
    end
  end
  -- (re)send
  local now = getTime()
  if busy and now - busy.sent > 150 then  -- 1.5 s without an answer
    if busy.tries >= 3 then
      if busy.kind == "find" then params[busy.name] = { missing = true } end
      setStatus("No answer from FC")
      busy = nil
    else
      busy.tries = busy.tries + 1
      busy.sent = now
      sendChunks(busy.cmd, busy.data)
    end
  end
  if not busy and #queue > 0 then
    busy = table.remove(queue, 1)
    busy.sent, busy.tries = now, 1
    rx = nil
    sendChunks(busy.cmd, busy.data)
  end
end

local function findAll()
  for _, pg in ipairs(PAGES) do
    for _, it in ipairs(pg.items) do
      local d = {}
      for i = 1, #it[2] do d[i] = string.byte(it[2], i) end
      queue[#queue + 1] = { cmd = CMD_FIND, data = d, name = it[2], kind = "find" }
    end
  end
end

local function sendSet(name, v)
  local p = params[name]
  local d = {}
  u16(d, p.idx); i32(d, v)
  queue[#queue + 1] = { cmd = CMD_SET, data = d, name = name, kind = "set" }
end

-- ---------------------------------------------------------------- screen
local function shown(it, v)
  if v == nil then return "..." end
  local x = v / 1000
  if it[2] == "vtx_band" then return BANDS[math.floor(x + 0.5)] or tostring(x) end
  if it[2] == "vtx_pit" then return x >= 0.5 and "ON" or "OFF" end
  if it[4] == 0 then return string.format("%d", math.floor(x + 0.5)) end
  return string.format("%." .. it[4] .. "f", x)
end

local function rows()
  local items = PAGES[page].items
  return #items + 1  -- + SAVE
end

local function draw()
  lcd.clear()
  local pg = PAGES[page]
  lcd.drawFilledRectangle(0, 0, LCD_W, 9)
  lcd.drawText(1, 1, "AllQuad " .. pg.title, SMLSIZE + INVERS)
  lcd.drawText(LCD_W - 1, 1, page .. "/" .. #PAGES, SMLSIZE + INVERS + RIGHT)
  local n = rows()
  local vis = 6
  if sel < top then top = sel end
  if sel > top + vis - 1 then top = sel - vis + 1 end
  for r = 0, vis - 1 do
    local k = top + r
    if k > n then break end
    local y = 11 + r * 8
    local inv = (k == sel) and INVERS or 0
    if k == n then
      lcd.drawText(1, y, "SAVE TO FC", SMLSIZE + inv)
    else
      local it = pg.items[k]
      local p = params[it[2]]
      lcd.drawText(1, y, it[1], SMLSIZE + ((k == sel and not editing) and INVERS or 0))
      local v = p and not p.missing and p.v or nil
      if k == sel and editing then v = editVal end
      local txt = (p and p.missing) and "--" or shown(it, v)
      lcd.drawText(LCD_W - 1, y, txt, SMLSIZE + RIGHT + ((k == sel and editing) and (INVERS + BLINK) or 0))
    end
  end
  local loading = 0
  for _, pg2 in ipairs(PAGES) do for _, it in ipairs(pg2.items) do if not params[it[2]] then loading = loading + 1 end end end
  local msg = loading > 0 and ("Reading " .. loading .. "...") or ((getTime() - statusT < 300) and status or "")
  if msg ~= "" then lcd.drawText(1, LCD_H - 7, msg, SMLSIZE) end
end

-- ---------------------------------------------------------------- keys
local function clampStep(it, p, dir)
  local v = editVal + dir * it[3] * 1000
  if v < p.min then v = p.min end
  if v > p.max then v = p.max end
  editVal = math.floor(v + 0.5)
end

local function init()
  findAll()
end

local function run(event)
  pump()
  local n = rows()
  local pg = PAGES[page]
  if editing then
    local it = pg.items[sel]
    local p = params[it[2]]
    if event == EVT_VIRTUAL_NEXT or event == EVT_VIRTUAL_INC then clampStep(it, p, 1)
    elseif event == EVT_VIRTUAL_PREV or event == EVT_VIRTUAL_DEC then clampStep(it, p, -1)
    elseif event == EVT_VIRTUAL_ENTER then
      editing = false
      if editVal ~= p.v then p.v = editVal; sendSet(it[2], editVal) end
    elseif event == EVT_VIRTUAL_EXIT then editing = false end
  else
    if event == EVT_VIRTUAL_EXIT then return 1 end
    if event == EVT_VIRTUAL_NEXT then sel = sel % n + 1
    elseif event == EVT_VIRTUAL_PREV then sel = (sel - 2) % n + 1
    elseif event == EVT_VIRTUAL_NEXT_PAGE or event == EVT_VIRTUAL_ENTER_LONG then page = page % #PAGES + 1; sel, top = 1, 1
    elseif event == EVT_VIRTUAL_PREV_PAGE then page = (page - 2) % #PAGES + 1; sel, top = 1, 1
    elseif event == EVT_VIRTUAL_ENTER then
      if sel == n then
        queue[#queue + 1] = { cmd = CMD_SAVE, data = {}, kind = "save" }
        setStatus("Saving...")
      else
        local p = params[pg.items[sel][2]]
        if p and not p.missing then editing = true; editVal = p.v end
      end
    end
  end
  draw()
  return 0
end

return { init = init, run = run }
