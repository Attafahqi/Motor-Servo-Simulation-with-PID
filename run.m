% Jalankan simulasi Simulink
sim('MotorServo.slx');  % Ganti 'nama_model_simulink' dengan nama model Simulinkmu

% Dapatkan informasi performansi sistem dari step response
sys_info = stepinfo(ans.y, ans.tout);

% Menampilkan informasi performansi
disp('Informasi Performansi Sistem:');
disp(sys_info);

