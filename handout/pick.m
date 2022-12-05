% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end