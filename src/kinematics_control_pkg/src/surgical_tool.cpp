#include "surgical_tool.hpp"

SurgicalTool::SurgicalTool() {
	init_surgicaltool(
		NUM_OF_JOINT,
		SEGMENT_ARC,
		ALPHA,
		SEGMENT_DIAMETER,
		WIRE_DISTANCE,
		);
	std::cout << "Surgical tool is created" << &this->surgicaltool_ << std::endl;
}

SurgicalTool::~SurgicalTool() {

}

void SurgicalTool::init_surgicaltool(int num_joint,
									 float arc,
                                     float alpha,
									 float diameter,
									 float disWire)
{
	this->surgicaltool_.num_joint = num_joint;
	this->surgicaltool_.arc 	  =	arc * mm_;
    this->surgicaltool_.alpha     =	alpha * mm_;
	this->surgicaltool_.diameter  =	diameter * mm_;
	this->surgicaltool_.disWire   =	disWire * mm_;
}

void SurgicalTool::set_bending_angle(double pAngle, double tAngle) {
	this->pAngle_ = pAngle * torad();
	this->tAngle_ = tAngle * torad();
}


std::tuple<double, double, double, double, double> SurgicalTool::get_bending_kinematic_result(
	double pAngle,
	double tAngle,
	)
{	
	// 1. set angle(degree) of continuum part
	this->set_bending_angle(pAngle, tAngle);
	// 3. calculate kinematics
	this->kinematics();

	return std::make_tuple(this->wrLengthEast_, this->wrLengthWest_, this->wrLengthSouth_, this->wrLengthNorth_, this->wrLengthGrip);
}

void SurgicalTool::kinematics()
{
// 라디안을 각도로 변환하는 함수
double rad2deg(double radian) {
    return radian * 180.0 / M_PI;
}

// 각도를 라디안으로 변환하는 함수
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

     // 각도의 절댓값을 구함
    double tilt_angle = std::abs(pAngle);
    double pan_angle = std::abs(tAngle);

    double disWire = 1.5;
    double p = pan_angle / (2 * num_joint); // deg
    double t = tilt_angle / (2 * num_joint); // deg
    double L = 3;

    double gamma = rad2deg(asin(disWire / (2 * arc)));
    double d_tr = 2 * arc * (1 - cos(deg2rad(t / 2 - gamma)));
    double d_pr = 2 * arc * (1 - cos(deg2rad(p / 2 - gamma)));
    

   /////////////////////////////// Pan motion tilt cable calculations////////////////////////////
    
    double a0[2] = {0, 0};
    double a1[2];
    if (pan_angle > 0)
        a1[0] = disWire / 2;
    else if (pan_angle == 0)
        a1[0] = 0;
  
    // 좌표 구하기
    
    double a2[2] = {a1[0], a1[1] + L};
    double a3[2] = {a2[0] + d_tr * sin(deg2rad(p)), a2[1] + d_tr * cos(deg2rad(p))};
    double a4[2] = {a3[0] + L * sin(deg2rad(2 * p)), a3[1] + L * cos(deg2rad(2 * p))};
    double a5[2] = {a4[0] + d_tr * sin(deg2rad(3 * p)), a4[1] + d_tr * cos(deg2rad(3 * p))};
    double a6[2] = {a5[0] + L * sin(deg2rad(4 * p)), a5[1] + L * cos(deg2rad(4 * p))};
    double a7[2] = {a6[0] + d_tr * sin(deg2rad(5 * p)), a6[1] + d_tr * cos(deg2rad(5 * p))};
    double a8[2] = {a7[0] + L * sin(deg2rad(6 * p)), a7[1] + L * cos(deg2rad(6 * p))};
    double a9[2] = {a8[0] + d_tr * sin(deg2rad(7 * p)), a8[1] + d_tr * cos(deg2rad(7 * p))};
    double a10[2] = {a9[0] + L * sin(deg2rad(8 * p)), a9[1] + L * cos(deg2rad(8 * p))};
    double a11[2] = {a10[0] + d_tr * sin(deg2rad(9 * p)), a10[1] + d_tr * cos(deg2rad(9 * p))};
    double a12[2] = {a11[0] + L * sin(deg2rad(10 * p)), a11[1] + L * cos(deg2rad(10 * p))};
    double a13[2] = {a12[0] - (disWire / 2) * sin(deg2rad(10 * p + 90)), a12[1] - (disWire / 2) * cos(deg2rad(10 * p + 90))};
    
    
    // contact point 각도
    double angle_deg0 = rad2deg(atan2(a13[1] - a0[1], a13[0] - a0[0]));
    double angle_deg1 = rad2deg(atan2(a2[1] - a0[1], a2[0] - a0[0]));
    double angle_deg2 = rad2deg(atan2(a4[1] - a0[1], a4[0] - a0[0]));
    double angle_deg3 = rad2deg(atan2(a6[1] - a0[1], a6[0] - a0[0]));
    
    
    // contact point 각도 선정을 위한 각도 비교
    double max_value = std::max({angle_deg0, angle_deg1, angle_deg2, angle_deg3});
    std::cout << "최대각도: " << max_value << std::endl;
    
    //Case 나누기
    double whole_cable_length;
    if (pan_angle >= 0) {
        if (max_value == angle_deg0) {
            whole_cable_length = sqrt(pow(a13[0] - a0[0], 2) + pow(a13[1] - a0[1], 2));
            std::cout << "case0\n";
        }else if (max_value == angle_deg3) {
            double cable_1 = sqrt(pow(a6[0] - a0[0], 2) + pow(a6[1] - a0[1], 2));
            whole_cable_length = d_tr + 2 * cable_1;
            std::cout << "case1\n";
        }else if (max_value == angle_deg2) {
            double cable_1 = sqrt(pow(a4[0] - a0[0], 2) + pow(a4[1] - a0[1], 2));
            whole_cable_length = 3 * d_tr + 2 * L + 2 * cable_1;
            std::cout << "case2\n";
        } else if (max_value == angle_deg1) {
            double cable_1 = sqrt(pow(a2[0] - a0[0], 2) + pow(a2[1] - a0[1], 2));
            whole_cable_length = 5 * d_tr + 4 * L + 2 * cable_1;
            std::cout << "case3\n";
        }
    }
    
    /////////////////////////////// tilt motion Pan cable calculations////////////////////////////
    
    double b0[2] = {0, 0};
    double b1[2];
    if (tilt_angle > 0)
        b1[0] = disWire / 2;
    else if (tilt_angle == 0)
        b1[0] = 0;
  
    // 좌표 구하기
    
    double b2[2] = {b1[0], b1[1] + L};
    double b3[2] = {b2[0] + d_pr * sin(deg2rad(t)), b2[1] + d_pr * cos(deg2rad(t))};
    double b4[2] = {b3[0] + L * sin(deg2rad(2 * t)), b3[1] + L * cos(deg2rad(2 * t))};
    double b5[2] = {b4[0] + d_pr * sin(deg2rad(3 * t)), b4[1] + d_pr * cos(deg2rad(3 * t))};
    double b6[2] = {b5[0] + L * sin(deg2rad(4 * t)), b5[1] + L * cos(deg2rad(4 * t))};
    double b7[2] = {b6[0] + d_pr * sin(deg2rad(5 * t)), b6[1] + d_pr * cos(deg2rad(5 * t))};
    double b8[2] = {b7[0] + L * sin(deg2rad(6 * t)), b7[1] + L * cos(deg2rad(6 * t))};
    double b9[2] = {b8[0] + d_pr * sin(deg2rad(7 * t)), b8[1] + d_pr * cos(deg2rad(7 * t))};
    double b10[2] = {b9[0] + L * sin(deg2rad(8 * t)), b9[1] + L * cos(deg2rad(8 * t))};
    double b11[2] = {b10[0] + d_pr * sin(deg2rad(9 * t)), b10[1] + d_pr * cos(deg2rad(9 * t))};
    double b12[2] = {b11[0] + L * sin(deg2rad(10 * t)), b11[1] + L * cos(deg2rad(10 * t))};
    double b13[2] = {b12[0] - (disWire / 2) * sin(deg2rad(10 * t + 90)), b12[1] - (disWire / 2) * cos(deg2rad(10 * t + 90))};
    
    
    // contact point 각도
    double angle_deg0_t = rad2deg(atan2(b13[1] - b0[1], b13[0] - b0[0]));
    double angle_deg1_t = rad2deg(atan2(b2[1] - b0[1], b2[0] - b0[0]));
    double angle_deg2_t = rad2deg(atan2(b4[1] - b0[1], b4[0] - b0[0]));
    double angle_deg3_t = rad2deg(atan2(b6[1] - b0[1], b6[0] - b0[0]));

    
    // contact point 각도 선정을 위한 각도 비교
    double max_value_t = std::max({angle_deg0_t, angle_deg1_t, angle_deg2_t, angle_deg3_t});
    std::cout << "최대각도: " << max_value_t << std::endl;
    
    //Case 나누기
    double whole_cable_length_t;
    if (tilt_angle >= 0) {
        if (max_value_t == angle_deg0_t) {
            whole_cable_length_t = sqrt(pow(b13[0] - b0[0], 2) + pow(b13[1] - b0[1], 2));
            std::cout << "case0\n";
        }else if (max_value_t == angle_deg3_t) {
            double cable_1_t = sqrt(pow(b6[0] - b0[0], 2) + pow(b6[1] - b0[1], 2));
            whole_cable_length_t = d_pr + 2 * cable_1_t;
            std::cout << "case1\n";
        }else if (max_value_t == angle_deg2_t) {
            double cable_1_t = sqrt(pow(b4[0] - b0[0], 2) + pow(b4[1] - b0[1], 2));
            whole_cable_length_t = 3 * d_pr + 2 * L + 2 * cable_1_t;
            std::cout << "case2\n";
        } else if (max_value_t == angle_deg1_t) {
            double cable_1_t = sqrt(pow(b2[0] - b0[0], 2) + pow(b2[1] - b0[1], 2));
            whole_cable_length_t = 5 * d_pr + 4 * L + 2 * cable_1_t;
            std::cout << "case3\n";
        }
    }
    
    double L_0 = 18.422904;
    /////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Tilt & Pancable delta calculations
    double delta_l_tl = whole_cable_length - L_0;
    double delta_l_tr = whole_cable_length - L_0;
    double delta_l_pl = whole_cable_length_t - L_0;
    double delta_l_pr = whole_cable_length_t - L_0;

    /////////////////////////////////////////////////마이너스 각도 고려///////////////////////////////////////////////////////
    
    double real_delta_l_pl, real_delta_l_pr, real_delta_l_tl, real_delta_l_tr;
    
    ////pr = East / pl = West / tr = South / tl = North


    // 1사분면(Pan '+' / Tilt'+')
    if (real_pan_angle >= 0 && real_tilt_angle >= 0) {
        this->wrLengthWest_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - p))) + delta_l_tl;
        this->wrLengthEast_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + p))) + delta_l_tr;
        this->wrLengthNorth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - t))) + delta_l_pl;
        this->wrLengthSouth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + t))) + delta_l_pr;
    }
    // 2사분면(Pan '-' / Tilt'+')
    else if (real_pan_angle < 0 && real_tilt_angle >= 0) {
        this->wrLengthWest_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + p))) + delta_l_tl;
        this->wrLengthEast_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - p))) + delta_l_tr;
        this->wrLengthNorth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - t))) + delta_l_pl;
        this->wrLengthSouth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + t))) + delta_l_pr;
    }
    // 3사분면(Pan '-' / Tilt'-')
    else if (real_pan_angle < 0 && real_tilt_angle < 0) {
        this->wrLengthWest_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + p))) + delta_l_tl;
        this->wrLengthEast_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - p))) + delta_l_tr;
        this->wrLengthNorth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + t))) + delta_l_pl;
        this->wrLengthSouth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - t))) + delta_l_pr;
    }
    // 4사분면(Pan '+' / Tilt'-')
    else if (real_pan_angle >= 0 && real_tilt_angle < 0) {
        this->wrLengthWest_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - p))) + delta_l_tl;
        this->wrLengthEast_  = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + p))) + delta_l_tr;
        this->wrLengthNorth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha + t))) + delta_l_pl;
        this->wrLengthSouth_ = 2 * surgicaltool_.num_joint * surgicaltool_.arc * (cos(deg2rad(alpha)) - cos(deg2rad(alpha - t))) + delta_l_pr;
    }

    // 결과 출력
   
	this->wrLengthEast_ =  this->wrLengthEast_ / mm_;
	this->wrLengthWest_ =  this->wrLengthWest_ / mm_;
	this->wrLengthSouth_ = this->wrLengthSouth_ / mm_;
	this->wrLengthNorth_ = this->wrLengthNorth_ / mm_;

    return 0;
}


float SurgicalTool::tomm()
{
	return this->mm_;
}

float SurgicalTool::torad()
{
	return this->deg_;
}