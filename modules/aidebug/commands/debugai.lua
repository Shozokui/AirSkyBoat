-----------------------------------
-- func: debugai
-- desc: Toggles debugging AI & Pathfinding for Entity
-- note: Must have the aidebug module enabled
-----------------------------------

cmdprops =
{
    permission = 1,
    parameters = "i"
}

function onTrigger(player)
  local targ
  targ = player:getCursorTarget()
  if not targ or not targ:isMob() then
    error(player, "You must either supply a mob ID or target a mob.")
    return
  end

  local enabled = targ:getLocalVar("AIDEBUG_ENABLED")
  if enabled == 1 then
    targ:setLocalVar("AIDEBUG_ENABLED", 0)
    player:setLocalVar("AIDEBUG_ENABLED", 0)
    player:PrintToPlayer("DebugAI: AI Debugging disabled for "..targ:getName())
  else
    targ:setLocalVar("AIDEBUG_ENABLED", 1)
    player:setLocalVar("AIDEBUG_ENABLED", 1)
    player:PrintToPlayer("DebugAI: AI Debugging enabled for "..targ:getName())
  end
end
