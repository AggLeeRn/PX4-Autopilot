local last = 0
local function init()
  setSerialBaudrate(115200)
end
local function run()
  local now = getTime()          -- 1/100 s
  if now - last >= 50 then       -- cca každou 0.5 s
    serialWrite("hello " .. now .. "\r\n")
    last = now
  end
  return 0                        -- 0 = běž dál
end
return { init=init, run=run }
