-- SENZOR.lua  ->  zkopiruj do  /SCRIPTS/FUNCTIONS/   (nazev max 6 znaku)
-- Cti pres log_telemetrie.py (posila validni JSON radky).
--
-- Sklada JSON jen pres ".." (sandbox nema table/string/math). Vse v pcall.
--
-- Krome telemetrie posila i polohu pacek jako "commanded" hodnoty:
--   cmd_roll  (aileron)  <-> skutecny Roll z dronu
--   cmd_pitch (elevator) <-> skutecny Ptch
--   cmd_yaw   (rudder)   <-> skutecny Yaw
--   cmd_thr   (throttle)
-- Packy jsou v procentech -100..+100 (surova hodnota -1024..+1024 delena 10.24).
-- Pozor: procenta packy nejsou stupne! Na porovnani "zadal jsem vs dron udelal"
-- si na PC muzes cmd_roll% preskalovat podle sveho limitu naklonu (napr. ANGLE_MAX).

local BAUD   = 115200
local PERIOD = 20       -- perioda vystupu v 1/100 s (20 = ~5 Hz)
local RESCAN = 200      -- jak casto znovu hledat senzory (200 = 2 s)

-- Telemetricke senzory: pouziji se jen ty, ktere na radiu existuji.
local CANDIDATES = {
  "RxBt","RSSI","RQly","TRSS","TPWR","TSNR","TQly",
  "VFAS","Curr","Fuel","Capa","Cels",
  "Alt","VSpd","GAlt","GSpd","Hdg","Sats","GPS",
  "Tmp1","Tmp2","Ptch","Roll","Yaw",
  "AccX","AccY","AccZ","RPM","Thr","IMUt","ESCt",
  "1RSS","2RSS","Fdev",
}

-- Packy: { klic do JSONu, nazev zdroje v getValue }
local STICKS = {
  { key = "cmd_roll",  src = "ail" },
  { key = "cmd_pitch", src = "ele" },
  { key = "cmd_yaw",   src = "rud" },
  { key = "cmd_thr",   src = "thr" },
}

local active   = {}
local lastOut  = 0
local lastScan = 0

local function scan()
  active = {}
  local n = 0
  for i = 1, #CANDIDATES do
    local fi = getFieldInfo(CANDIDATES[i])
    if fi then
      n = n + 1
      active[n] = { name = CANDIDATES[i], id = fi.id }
    end
  end
end

local function build()
  local s = '{"t":' .. getTime()

  -- telemetricke senzory
  for i = 1, #active do
    local sen = active[i]
    local v = getValue(sen.id)
    if type(v) == "table" then
      if v.lat and v.lon then
        s = s .. ',"lat":' .. v.lat .. ',"lon":' .. v.lon
      end
    elseif type(v) == "number" then
      s = s .. ',"' .. sen.name .. '":' .. v
    end
  end

  -- packy (commanded), preskalovane na procenta -100..+100
  for i = 1, #STICKS do
    local sv = getValue(STICKS[i].src)
    if type(sv) == "number" then
      s = s .. ',"' .. STICKS[i].key .. '":' .. (sv / 10.24)
    end
  end

  return s .. "}"
end

local function init()
  setSerialBaudrate(BAUD)
  scan()
end

local function loop()
  local now = getTime()
  if now - lastScan >= RESCAN then
    lastScan = now
    scan()
  end
  if now - lastOut >= PERIOD then
    lastOut = now
    local ok, line = pcall(build)
    if ok then
      serialWrite(line .. "\r\n")
    else
      serialWrite('{"err":"' .. tostring(line) .. '"}\r\n')
    end
  end
  return 0
end

return { init = init, run = loop, background = loop }
