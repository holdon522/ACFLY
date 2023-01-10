                case 0:
                {
                    if(distance<50){
                    Position_Control_set_TargetVelocityXY_AngleLimit(0,15,0,3);
                    }
                    else {
                        Position_Control_set_XYLock();
							Position_Control_set_ZLock();
                        ++mission_ind;
                    }
                }
                case 1:
					{	//起飞
						double params[7];
						params[0] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 40;
						params[6] = 0;
	 					int16_t res = Process_NavCmd( MAV_CMD_NAV_TAKEOFF, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//起飞完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
                    case 2:
                    {
                    Position_Control_set_ZLock();
					Position_Control_set_XYLock();
					Attitude_Control_set_Target_YawRelative(degree2rad(-90.0));

                    }