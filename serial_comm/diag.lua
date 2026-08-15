-- DIAG.lua  ->  zkopiruj do  /SCRIPTS/FUNCTIONS/
-- POZOR: cti to SYROVOU cteckou (ta s ser.read(200) a print(repr)),
-- NE pres log_telemetrie.py -- tyhle hlasky nejsou JSON a log by je schoval.

local last = 0

local function init()
  setSerialBaudrate(115200)
  serialWrite("DIAG: init ok\r\n")          -- kdyz tohle prijde, skript se nacetl a bezi
end

local function loop()
  local now = getTime()
  if now - last >= 50 then                  -- cca 2x za sekundu
    last = now
    local ok, err = pcall(function()
      local fi = getFieldInfo("RxBt")       -- tohle je podezrele misto
      serialWrite("DIAG: tick t=" .. now ..
                  " getFieldInfo=" .. tostring(fi) .. "\r\n")
    end)
    if not ok then
      serialWrite("DIAG: ERROR " .. tostring(err) .. "\r\n")  -- ukaze presnou chybu
    end
  end
  return 0
end

return { init = init, run = loop, background = loop }
