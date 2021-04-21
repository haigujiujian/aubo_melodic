#include <iostream>
#include<Eigen/Core>
//Makes sure minimal angle is 0
double regularAngle(double theta)
{           if (theta < ZERO_THRESH && theta > -ZERO_THRESH)
            {
                        theta = 0;
            }
           if(std::isnan(theta))theta = 0;
           return theta;
}
//SDH general trans matrix
Eigen::Matrix<double,4,4> TransMatrixSDH(double a, double alpha, double d, double theta) 
{
                Eigen::Matrix<double,4,4> T;
                T.row(0)<<cos(theta),-sin(theta)*cos(alpha), sin(theta)*sin(alpha),a*cos(theta);
                T.row(1)<<sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
                T.row(2)<<0,sin(alpha),cos(alpha),d;
                T.row(3)<<0,0,0,1;
                 return T;
}
//Inverse Kinematics- Mixed method
int InverseKinematics(Eigen::Matrix<double,4,4> TcpPOS, double **q, double q6Des)
{
                int numSols = 0;
                double q1[2];
                double q5[2][2];
                double q6[2][2];
                double q3[2][2][2];
                double q2[2][2][2];
                double q4[2][2][2];
                //==========================================
                //Solving for shoulder_joint q1
                //=========================================
                //the location of the fifth coordinate frame wrt the base:
                Eigen::Vector4d P0_5 = TcpPOS * Eigen::Vector4d(0, 0, -d6, 1) - Eigen::Vector4d(0,0, 0, 1);
                double psi = atan2(P0_5[1], P0_5[0]);
                std::cout<<"psi:"<<psi<<std::endl;
                double phi_pos;
                if((fabs(d4/(sqrt(pow(P0_5[0],2) +pow(P0_5[1],2))))>=1)&&(fabs(d4 /(sqrt(pow(P0_5[0],2) + pow(P0_5[1],2))))<(1+ZERO_THRESH)))
                {phi_pos=0;}
                else
                phi_pos = acos(d4 / (sqrt(pow(P0_5[0],2) + pow(P0_5[1],2))));
                std::cout<<"d4/sqrtvalue:"<<d4/(sqrt(pow(P0_5[0],2)+pow(P0_5[1],2)))<<std::endl;
                std::cout<<"phi_pos:"<<phi_pos<<std::endl;
                q1[0] = regularAngle(psi + phi_pos + M_PI/2);
                q1[1] = regularAngle(psi - phi_pos + M_PI/2);
                std::cout<<"q1[0]:"<<psi + phi_pos + M_PI/2<<" q1[1]:"<<psi - phi_pos +M_PI/2<<std::endl;
                //==========================================
                //Solving for wrist2_joint q5
                //=========================================
                //the location of the sixth coordinate frame wrt the base:
                //Eigen::Vector4d P0_6 = TcpPOS * Eigen::Vector4d(0, 0, 0, 1) - Eigen::Vector4d(0,0, 0, 1);
                //For each solution to q1
                for (int i = 0; i < 2; i++)
                {
                        //find transformation from frame 6 to frame 1
                        Eigen::Matrix<double,4,4> T1_0 =(TransMatrixSDH(a1,q1[i])).inverse();
                        Eigen::Matrix<double,4,4> T1_6 = T1_0* TcpPOS;
                        //the z location of the 6th frame wrt the 1st frame
                        //double P1_6z = P0_6[0]*sin(q1[i]) - P0_6[1]*cos(q1[i]);
                        double P1_6z = T1_6(2,3);
                        double temp_trig_ratio = (P1_6z - d4) / d6;
                        std::cout<<"temp_trig_ratio:"<<temp_trig_ratio<<std::endl;
                        //invalid if the argument to acos is not in [-1, 1]:
                        if((fabs(temp_trig_ratio)>=1)&&(fabs(temp_trig_ratio)<(1+ZERO_THRESH))){temp_trig_ratio=1.0;}
                        if (fabs(temp_trig_ratio) > 1.0)  continue;
                        q5[i][0] = regularAngle(acos(temp_trig_ratio));
                        q5[i][1] = regularAngle(-acos(temp_trig_ratio));
                        std::cout<<"q5["<<i<<"][0]:"<<q5[i][0]<<"q5["<<i<<"][1]:"<<q5[i][1]<<std::endl;
                        //==========================================
                        //Solving for q6
                        //=========================================
                        Eigen::Matrix<double,4,4> T0_1 = TransMatrixSDH(a1, alpha1, d1, q1[i]);
                        Eigen::Matrix<double,4,4> T6_1 = TcpPOS.inverse()*T0_1;
                        //For each solution to q5
                        for(int j = 0; j < 2; j++)
                        {
                                    double sin_q5 = sin(q5[i][j]);
                                    double z6_1_y = T6_1(1,2);
                                    double z6_1_x = T6_1(0,2);
                                    //invalid if sin(q5)=0 or z_x and z_y both =0, in this case q6 is free
                                    if ( (sin_q5 < ZERO_THRESH && sin_q5 > -ZERO_THRESH) ||( (z6_1_x < ZERO_THRESH && z6_1_x > -ZERO_THRESH) &&(z6_1_y < ZERO_THRESH && z6_1_y > -ZERO_THRESH) ) )
                                    {
                                          q6[i][j] = 0; //choose arbitrary q6
                                    }
                                     else
                                    {
                                          q6[i][j] = regularAngle(atan2( -z6_1_y / sin_q5, z6_1_x / sin_q5 ));
                                    }
                                    std::cout<<"q6[i][j]"<<q6[i][j]<<std::endl;
                                    /*==========================================
                                    /Solving for q2-q4
                                    /=========================================*/
                                    //find location of frame 3 wrt frame 1
                                    Eigen::Matrix<double,4,4> T5_4 = (TransMatrixSDH(a5, alpha5, d5,q5[i][j])).inverse();
                                    Eigen::Matrix<double,4,4> T6_5 = (TransMatrixSDH(a6, alpha6, d6,q6[i][j])).inverse();
                                    Eigen::Matrix<double,4,4> T1_4 = T1_0 * TcpPOS * T6_5 * T5_4;
                                    Eigen::Vector4d P1_3 = T1_4 * Eigen::Vector4d(0, -d4, 0, 1) -Eigen::Vector4d(0,0,0,1);
                                    //solve for q3 first
                                    temp_trig_ratio = (pow(P1_3[0],2) + pow(P1_3[1],2) - pow(a2, 2) -pow(a3, 2)) / (2 * a2 * a3);
                                    if((fabs(temp_trig_ratio)>=1)&&(fabs(temp_trig_ratio)<(1+ZERO_THRESH)))
                                                {temp_trig_ratio=1.0;}
                                    //invalid if the argument to acos is not in [-1, 1]:
                                    if (fabs(temp_trig_ratio) > 1) continue;
                                    double theta3 = acos(temp_trig_ratio);
                                    q3[i][j][0] = regularAngle(theta3);
                                    q3[i][j][1] = regularAngle(-theta3);
                                    std::cout<<"q3[i][j]"<<q3[i][j][0]<<" "<<q3[i][j][1]<<std::endl;
                                    //For each solution to q3
                                    for(int k = 0; k < 2; k++)
                                    {
                                    //solve for q2
                                    q2[i][j][k] = regularAngle(-atan2(P1_3(1), -P1_3(0)) + asin(a3 *sin(q3[i][j][k]) / sqrt(pow(P1_3[0],2) + pow(P1_3[1],2))));
                                    //find transformation from frame 3 to 4
                                    Eigen::Matrix<double,4,4> T1_2 = TransMatrixSDH(a2, alpha2, d2,q2[i][j][k]);
                                    Eigen::Matrix<double,4,4> T2_3 = TransMatrixSDH(a3, alpha3, d3,q3[i][j][k]);
                                    Eigen::Matrix<double,4,4> T3_4 = (T1_2 * T2_3).inverse() * T1_4;
                                    //extract q4 from it
                                    q4[i][j][k] = regularAngle(atan2(T3_4(1,0), T3_4(0,0)));
                                    std::cout<<"q4[i][j][k]"<<q4[i][j][k]<<std::endl;
                                    //write joint angles to solution buffer
                                    q[numSols][0] = q1[i];
                                    q[numSols][1] = q2[i][j][k];
                                    q[numSols][2] = q3[i][j][k];
                                    q[numSols][3] = q4[i][j][k];
                                    q[numSols][4] = q5[i][j];
                                    q[numSols][5] = q6[i][j];
                                    numSols++;
                                    }
                         }
                }
                return numSols;
}