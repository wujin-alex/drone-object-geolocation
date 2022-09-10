#include "drone_objlocation.h"

using namespace std;
using namespace Eigen;

DroneObjlocation::DroneObjlocation()
{
}

/**
 * @brief 设置相机参数，推荐对相机提前校准
 * @param img_width    相机图像宽度
 * @param img_height   相机图像高度
 * @param focal        一倍变焦时的焦距值，单位mm
 * @param fx   fx，像素焦距，如果没有校准，fx=focal/sigma，其中sigma为像元大小
 * @param fy   fy，如果没有校准则与fx一样
 * @param cx   cx，如果没有校准，cx=img_width/2
 * @param cy   cy，如果没有校准，cy=img_height/2
 */
DroneObjlocation::DroneObjlocation(uint16_t img_width, uint16_t img_height, float focal, float fx, float fy, float cx, float cy)
{
    _focal_base = focal;     //mm
    _img_width  = img_width;
    _img_height = img_height;
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
}

Eigen::Matrix3d DroneObjlocation::get_K()
{
    return K;
}
Eigen::Matrix3d DroneObjlocation::get_K(float focal)
{
    Matrix3d K;
    // 根据焦距最小时校正的参数获得变焦后的K矩阵参数
    float zoom = focal / _focal_base;
    K << _fx*zoom, 0, _cx, 0, _fy*zoom, _cy, 0, 0, 1;
    return K;
}

/**
 * @brief 设置相机参数
 * @param img_width    相机图像宽度
 * @param img_height   相机图像高度
 * @param focal        一倍变焦时的焦距值，单位mm
 * @param fx   fy，像素焦距
 * @param fy   fx，像素焦距
 * @param cx  cx
 * @param cy  cy
 */
void DroneObjlocation::set_parameter(uint16_t img_width, uint16_t img_height, float focal, float fx, float fy, float cx, float cy)
{
    _focal_base = focal;     //mm
    _img_width  = img_width;
    _img_height = img_height;
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
}

/**
 * @brief 根据目标相机坐标系下坐标计算目标在机体坐标系下的坐标
 * @param uv             目标所在图像中的像素坐标ux,vy
 * @param focal          相机焦距
 * @param distance       激光测距，即吊舱与图像中心对应的地面目标之间的距离
 * @param euler_camera   吊舱姿态角roll,pitch,yaw
 * @param euler_drone    无人机姿态角roll,pitch,yaw
 * @param position_drone 无人机经纬高坐标(lat,lon,alt)
 * @param distance_type  0-distance代表无人机距离起飞点垂直高度，1-distance代表激光测距
 * @return 目标在相机、机体、站点NED坐标系下的坐标
 */
std::map<std::string, std::vector<float>> DroneObjlocation::get_target_location(std::vector<int> uv, float focal, float distance, 
                                                                            std::vector<float> euler_camera, 
                                                                            std::vector<float> euler_drone, 
                                                                            std::vector<float> position_drone,
                                                                            int distance_type)
{
    std::map<std::string, std::vector<float>> results = {{"p_c", {}}, {"p_b", {}}, {"p_e", {}}, {"gps", {}}};

    /* 1-1. 根据焦距求K矩阵 */
    Matrix3d K;
    // 根据焦距最小时校正的参数获得变焦后的K矩阵参数
    float zoom = focal / _focal_base;
    K << _fx*zoom, 0, _cx, 0, _fy*zoom, _cy, 0, 0, 1;

    /* 1-2. 准备转移矩阵 */
    float roll_c  = euler_camera[0];
    float pitch_c = euler_camera[1];
    float yaw_c   = euler_camera[2];
    float roll_b  = euler_drone[0];
    float pitch_b = euler_drone[1];
    float yaw_b   = euler_drone[2];

    Matrix3d Rcb, Rbe;
    Rcb = AngleAxisd(yaw_c, Vector3d::UnitZ()) * AngleAxisd(pitch_c, Vector3d::UnitY()) * AngleAxisd(roll_c, Vector3d::UnitX());
    Rbe = AngleAxisd(yaw_b, Vector3d::UnitZ()) * AngleAxisd(pitch_b, Vector3d::UnitY()) * AngleAxisd(roll_b, Vector3d::UnitX());
    // cout << "Rcb=\n" << Rcb << endl;
    // cout << "Rbe=\n" << Rbe << endl;

    /* 1-3. 参考向量，在站点NED坐标系下与z轴同方向 */
    Vector3d pref_g(0, 0, 1);

    /* 2. 求无人机距离目标所在水平平面垂直高度
     * 若开启了激光测距，根据激光测距获取
     * 若没有开启激光测距，则假设大地水平，根据无人机距离起飞点高度作为该垂直高度的估计值
     */
    float H, L, theta_1;
    Vector3d p0_c, p0_e;
    switch(distance_type)
    {
        case 0:
            H = distance;
            break;
        case 1:
            // 激光测距距离，机无人机与图像中心对应的地面位置之间的距离为L=distance
            L = distance;
            // p0在相机坐标系下的坐标
            p0_c = Vector3d(L, 0, 0);
            // p0在站点NED坐标系下的坐标
            p0_e = Rbe * Rcb * p0_c;
            // 归一化
            p0_e.normalize();
            // 求与参考向量的夹角theta_1
            theta_1 = asin(p0_e.cross(pref_g).norm());
            // 得到距离目标所在水平平面垂直高度
            H = L * cos(theta_1);
            // cout << "theta_1, in rad=" << theta_1 << ", in deg=" << theta_1 * 180 / M_PI << endl;
            // cout << "H=" << H << endl;
            break;
    }

    /* 3. 求相机到目标target的距离 */
    // 目标在图像坐标系下的坐标
    Vector3d pt_uv(uv[0], uv[1], 1);
    // 图像坐标系到归一化平面坐标系
    Vector3d pt_norm;
    pt_norm = K.inverse() * pt_uv;
    // cout << "pt_norm=" << pt_norm << endl;
    // 归一化平面坐标系到相机坐标系
    Vector3d pt_c_norm(pt_norm.z(), pt_norm.x(), pt_norm.y());
    // 相机坐标系到站点NED坐标系
    Vector3d pt_e_norm = Rbe * Rcb * pt_c_norm;
    pt_e_norm.normalize();
    // 求与参考向量夹角theta_2
    float theta_2 = asin(pt_e_norm.cross(pref_g).norm());
    // 求距离
    float Lt = H / cos(theta_2);
    // cout << "pt_c_norm=" << pt_c_norm << endl;

    /* 4-1. 求目标在相机坐标系下坐标 */
    Vector3d pt_c = pt_c_norm / pt_c_norm.norm() * Lt;
    // cout << "pt_c=" << pt_c << endl;

    /* 4-2. 求目标在机体坐标系下坐标 */
    Vector3d pt_b = Rcb * pt_c;
    // cout << "pt_b=" << pt_b << endl;

    /* 4-3. 求目标在站点NED坐标系下坐标 */
    Vector3d pt_e = Rbe * pt_b;
    // cout << "pt_e=" << pt_e << endl;

    /* 4-4. 求无人机在ECEF坐标系下坐标（即站点ECEF坐标） */
    Vector3d drone_ecef = LatLonAlt2ECEF(Vector3d(position_drone[0], position_drone[1], position_drone[2]));

    /* 4-5. 求目标在ECEF坐标系下坐标 */
    Matrix3d R_ned2ecef = CalRotation_NED2ECEF(Vector3d(position_drone[0], position_drone[1], position_drone[2]));
    Vector3d pt_ecef    = R_ned2ecef * pt_e + drone_ecef;

    /* 4-6. 求目标经纬高 */
    // 方法1
    Vector3d pt_lla = ECEF2LatLonAlt(pt_ecef);
    // cout << "pt_lla=" << pt_lla << endl;
    // 方法2
    // Vector3d drone_gps(position_drone[0], position_drone[1], position_drone[2]);
    // Vector3d pt_lla = convert_position_local2global(drone_gps, pt_e);


    /* */
    results["p_c"] = {(float)(pt_c.x()), (float)(pt_c.y()), (float)(pt_c.z())};
    results["p_b"] = {(float)(pt_b.x()), (float)(pt_b.y()), (float)(pt_b.z())};
    results["p_e"] = {(float)(pt_e.x()), (float)(pt_e.y()), (float)(pt_e.z())};
    results["gps"] = {(float)(pt_lla.x()), (float)(pt_lla.y()), (float)(pt_lla.z())};
    return results;
}


Vector3d DroneObjlocation::calculate_second_gps_ned(Vector3d gps1, float distance, float angle)
{
	/*根据目标点1相关数据计算目标点2GPS坐标*/
	Vector3d gps2;
	float R = 6371000.0;
	double lat1 = gps1.x() / Rad_to_deg;
	double lon1 = gps1.y() / Rad_to_deg;
    double alt1 = gps1.z();
	double pi_half = M_PI / 2.0;
	double c = distance / R;

	double a = acos(cos(pi_half - lat1)*cos(c) + sin(pi_half - lat1)*sin(c) * cos(angle));
	double C = asin(sin(c)*sin(angle) / sin(a));

	double lat2 = (pi_half - a) * Rad_to_deg;
	double lon2 = (lon1 + C) * Rad_to_deg;

	gps2 << lat2, lon2, alt1;
	return gps2;
}

/**
* @brief 根据目标在局部坐标系的坐标与无人机当前的经纬度计算目标的经纬度
* @param origin_local	  无人机当前经纬度
* @param local_positions  目标在局部坐标系下的坐标（默认设置只存在旋转）
* @return 目标的经纬度
*/
Vector3d DroneObjlocation::convert_position_local2global(Vector3d origin_local, Vector3d local_positions)
{
	float l_x = local_positions.x();
	float l_y = local_positions.y();
	Vector3d gps_lat;
	Vector3d gps_lon;
	Vector3d gps_return;
	l_x >= 0 ? gps_lat = DroneObjlocation::calculate_second_gps_ned(origin_local, l_x, 0) : gps_lat = DroneObjlocation::calculate_second_gps_ned(origin_local, -l_x, M_PI);
	l_y >= 0 ? gps_lon = DroneObjlocation::calculate_second_gps_ned(origin_local, l_y, M_PI / 2) : gps_lon = DroneObjlocation::calculate_second_gps_ned(origin_local, -l_y, -M_PI / 2);
	float lat = gps_lat.x();
	float lon = gps_lon.y();
    float alt = origin_local.z() + local_positions.z();
	gps_return << lat, lon, alt;
	return gps_return;
}

/**
* @brief 根据经纬高计算对应的ECEF坐标
* @param pointLLA	 经纬高（wgs84）
* @return ECEF坐标
*/
Vector3d DroneObjlocation::LatLonAlt2ECEF(Vector3d pointLLA)
{
    Vector3d pointECEF(0.0, 0.0, 0.0);

    double lat = pointLLA.x() * M_PI / 180.0;
    double lon = pointLLA.y() * M_PI / 180.0;
    double alt = pointLLA.z();

    // 基准椭球体的长半径
    double constA = 6378137.0;

    // 基准椭球体的极扁率, f=a-ba
    double constF = 1 / 298.257223565;

    // 基准椭球体的曲率半径
    double constN = constA / sqrt(1 - constF * (2 - constF) * pow(sin(lat), 2));

    pointECEF.x() = (constN + alt) * cos(lat) * cos(lon);
    pointECEF.y() = (constN + alt) * cos(lat) * sin(lon);
    pointECEF.z() = (constN * pow(1 - constF, 2) + alt) * sin(lat);

    return pointECEF;
}

/**
* @brief 根据ECEF坐标计算对应的经纬高
* @param pointECEF	 ECEF坐标
* @return 经纬高（wgs84）
*/
Vector3d DroneObjlocation::ECEF2LatLonAlt(Vector3d pointECEF)
{
    Vector3d pointLLA(0, 0, 0);

    double ecefX = pointECEF.x();
    double ecefY = pointECEF.y();
    double ecefZ = pointECEF.z();

    // 基准椭球体的长半径
    double constA = 6378137.0;

    // 基准椭球体的极扁率
    double constF = 1 / 298.257223565;

    // WGS-84下偏心率
    double ConstEPow2 = constF * (2 - constF);

    // lambda
    int kCount = 0;
    double latNew = 0;
    double latLast = 5201314;
    double altLast = 5201314;
    double lon = 0;
    double altNew = 0;
    while (!((abs(latLast - latNew) < 1e-12 && abs(altLast - altNew) < 1e-12) || kCount > 100))
    {
        //std::cout << "k = " << kCount << "; error = " << abs(latLast - latN) << std::endl;

        latLast = latNew;
        altLast = altNew;
        // 基准椭球体的曲率半径
        double tempN = constA / sqrt(1 - ConstEPow2 * pow(sin(latNew), 2));

        lon = atan2(ecefY, ecefX);
        double tempP = sqrt(pow(ecefX, 2) + pow(ecefY, 2));
        altNew = tempP / cos(latNew) - tempN;
        latNew = atan2(ecefZ / tempP, (1 - ConstEPow2*tempN / (tempN + altNew)));
        kCount++;
        //std::cout << "kCoutnt" << kCount << std::endl;
    }

    pointLLA.x() = latNew * 180.0 / M_PI;
    pointLLA.y() = lon * 180.0 / M_PI;
    pointLLA.z() = altNew;

    // if (abs(pointLLA.x()) > 90.0 || abs(pointLLA.y()) > 180.0)
    // {
    //     return false;
    // }

    return pointLLA;
}


/**
* @brief 计算站点NED坐标系到ECEF坐标系的转移矩阵
* @param pointLLA	 站点的经纬高坐标
* @return 旋转矩阵，站点NED坐标系到ECEF坐标系
*/
Matrix3d DroneObjlocation::CalRotation_NED2ECEF(Vector3d pointLLA)
{
    Matrix3d R_ned2ecef, R_o;
    R_o << 0,0,-1,
           0,1,0,
           1,0,0;

    double lat = pointLLA.x() * M_PI / 180.0;
    double lon = pointLLA.y() * M_PI / 180.0;

    R_ned2ecef = AngleAxisd(lon, Vector3d::UnitZ()) * AngleAxisd(-lat, Vector3d::UnitY()) * R_o;

    // 等同于
    // R_ned2ecef << -cos(lon)*sin(lat),  -sin(lon),  -cos(lon)*cos(lat),
    //               -sin(lon)*sin(lat),  cos(lon),   -sin(lon)*cos(lat),
    //               cos(lat),            0,          -sin(lat);

    return R_ned2ecef;
}
