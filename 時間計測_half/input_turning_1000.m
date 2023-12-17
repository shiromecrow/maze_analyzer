function [speed1000_shortest_mollifier]=input_turning_1000



		speed1000_shortest_mollifier.SlalomCentervelocity = 1000;
		speed1000_shortest_mollifier.TurnCentervelocity = 1000;

		speed1000_shortest_mollifier.slalom_R.g_speed =speed1000_shortest_mollifier.SlalomCentervelocity;
		speed1000_shortest_mollifier.slalom_R.t_speed = 1100;
		speed1000_shortest_mollifier.slalom_R.f_ofset = 5;
		speed1000_shortest_mollifier.slalom_R.e_ofset = 28;

		speed1000_shortest_mollifier.slalom_L.g_speed =speed1000_shortest_mollifier.SlalomCentervelocity;
		speed1000_shortest_mollifier.slalom_L.t_speed = 1100;
		speed1000_shortest_mollifier.slalom_L.f_ofset = 5;
		speed1000_shortest_mollifier.slalom_L.e_ofset = 32;

		speed1000_shortest_mollifier.turn90_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn90_R.t_speed = 1450;
		speed1000_shortest_mollifier.turn90_R.f_ofset = 31;
		speed1000_shortest_mollifier.turn90_R.e_ofset = 31;

		speed1000_shortest_mollifier.turn90_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn90_L.t_speed = 1450;
		speed1000_shortest_mollifier.turn90_L.f_ofset = 33;
		speed1000_shortest_mollifier.turn90_L.e_ofset = 35;

		speed1000_shortest_mollifier.turn180_R.g_speed =1000;
		speed1000_shortest_mollifier.turn180_R.t_speed = 1400;
		speed1000_shortest_mollifier.turn180_R.f_ofset = 17;
		speed1000_shortest_mollifier.turn180_R.e_ofset = 20;

		speed1000_shortest_mollifier.turn180_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn180_L.t_speed = 1400;
		speed1000_shortest_mollifier.turn180_L.f_ofset = 17;
		speed1000_shortest_mollifier.turn180_L.e_ofset = 18;

		speed1000_shortest_mollifier.turn45in_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn45in_R.t_speed = 1300;
		speed1000_shortest_mollifier.turn45in_R.f_ofset = 20;
		speed1000_shortest_mollifier.turn45in_R.e_ofset = 38;

		speed1000_shortest_mollifier.turn45in_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn45in_L.t_speed = 1300;
		speed1000_shortest_mollifier.turn45in_L.f_ofset = 19;
		speed1000_shortest_mollifier.turn45in_L.e_ofset = 36;

		speed1000_shortest_mollifier.turn135in_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn135in_R.t_speed = 1590;
		speed1000_shortest_mollifier.turn135in_R.f_ofset = 17;
		speed1000_shortest_mollifier.turn135in_R.e_ofset = 15;

		speed1000_shortest_mollifier.turn135in_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn135in_L.t_speed = 1560;
		speed1000_shortest_mollifier.turn135in_L.f_ofset = 15;
		speed1000_shortest_mollifier.turn135in_L.e_ofset = 12;

		speed1000_shortest_mollifier.turn45out_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn45out_R.t_speed = 1300;
		speed1000_shortest_mollifier.turn45out_R.f_ofset = 39;
		speed1000_shortest_mollifier.turn45out_R.e_ofset = 22;

		speed1000_shortest_mollifier.turn45out_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn45out_L.t_speed = 1300;
		speed1000_shortest_mollifier.turn45out_L.f_ofset = 31;
		speed1000_shortest_mollifier.turn45out_L.e_ofset = 24;

		speed1000_shortest_mollifier.turn135out_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn135out_R.t_speed = 1660;
		speed1000_shortest_mollifier.turn135out_R.f_ofset = 17;
		speed1000_shortest_mollifier.turn135out_R.e_ofset = 22;

		speed1000_shortest_mollifier.turn135out_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn135out_L.t_speed = 1660;
		speed1000_shortest_mollifier.turn135out_L.f_ofset = 11;
		speed1000_shortest_mollifier.turn135out_L.e_ofset = 30;

		speed1000_shortest_mollifier.V90_R.g_speed = 1000;
		speed1000_shortest_mollifier.V90_R.t_speed = 1750;
		speed1000_shortest_mollifier.V90_R.f_ofset = 16.5;
		speed1000_shortest_mollifier.V90_R.e_ofset = 23;

		speed1000_shortest_mollifier.V90_L.g_speed = 1000;
		speed1000_shortest_mollifier.V90_L.t_speed = 1750;
		speed1000_shortest_mollifier.V90_L.f_ofset = 14;
		speed1000_shortest_mollifier.V90_L.e_ofset = 23;

end
