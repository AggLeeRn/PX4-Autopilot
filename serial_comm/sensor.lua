-- SENZOR.lua  ->  zkopiruj do  /SCRIPTS/FUNCTIONS/   (nazev max 6 znaku)
-- Cti pres log_telemetrie.py (posila validni JSON radky).
--
-- POZNAMKA: sandbox funkcnich skriptu na tomhle radiu NEMA knihovny
-- table / string / math. Proto se JSON sklada jen pres ".." a cisla se
-- prevadeji na text automaticky (s teckou, jak JSON vyzaduje).
-- Zadne table.concat / string.format / math.floor.
--
-- Bez pripojeneho dronu posila jen {"t":...}. Az das "Discover new sensors",
-- do radku se samy pridaji dalsi klice. Vse je v pcall -> pri chybe {"err":...}.

local BAUD   = 115200
local PERIOD = 20       -- perioda vystupu v 1/100 s (20 = ~5 Hz)
local RESCAN = 200      -- jak casto znovu hledat senzory (200 = 2 s)

-- Pouziji se jen ty, ktere na radiu existuji. Pripadne dopis dalsi jmena
-- presne tak, jak je vidis v Model > Telemetry.
local CANDIDATES = {
  "RxBt","RSSI","RQly","TRSS","TPWR","TSNR","TQly","RSNR","ANT","RFMD","Bat%",
  "VFAS","Curr","Fuel","Capa","Cels",
  "Alt","VSpd","GAlt","GSpd","Hdg","Sats","GPS",
  "Tmp1","Tmp2","Ptch","Roll","Yaw",
  "AccX","AccY","AccZ","RPM","Thr","IMUt","ESCt",
  "1RSS","2RSS","Fdev",
}

local active   = {}     -- tabulky jako {} funguji, chybi jen knihovna 'table'
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

-- poskladá JSON radek jen spojovanim retezcu (bez knihoven); vola se pres pcall
local function build()
  local s = '{"t":' .. getTime()
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
