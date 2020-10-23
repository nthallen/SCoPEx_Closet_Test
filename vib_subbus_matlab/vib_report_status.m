function vib_report_status(s)
vib_base = 5*16;
status = read_subbus(s, vib_base);
if bitand(status,9) == 9
  fprintf(1, 'Status: Vib sensor initialized, RESET mode\n');
elseif bitand(status,1)
  fprintf(1, 'Status: Vib sensor initialized, ');
  if bitand(status,16)
    fprintf(1, 'PRESET mode\n');
  elseif bitand(status,32)
    fprintf(1, 'ACTIVE mode\n');
  else
    fprintf(1, 'NO MODE bits\n');
  end
else
  fprintf(1, 'Status: Vib sensor not detected\n');
end
if bitand(status, 2)
  fprintf(1, 'WARNING: Observed FIFO_OVFLOW\n');
end
if bitand(status,4)
  fprintf(1,'INFO: Observed DUPLICATE\n');
end
