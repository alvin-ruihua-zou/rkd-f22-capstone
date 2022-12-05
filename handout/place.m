% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end