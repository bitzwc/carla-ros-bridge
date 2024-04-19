
#include <carla_shenlan_a_star_planner/random_map_generator.h>

// using namespace std;
// using namespace Eigen;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("random_map_generator");

RandomMapGenerator::RandomMapGenerator() : Node("random_map_generator"){
    _init_x = 0.0;
    _init_y = 0.0;

    _x_size = 40.0;
    _y_size = 40.0;
    _z_size = 0.0;

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    this->RandomMapGenerate();

    _all_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 10);

    //每500ms发布一次点云地图
    global_map_publish_timer = this->create_wall_timer(500ms, std::bind(&RandomMapGenerator::pubSensedPoints, this));
}
RandomMapGenerator::~RandomMapGenerator() {}

void RandomMapGenerator::RandomMapGenerate() {
    //随机种子生成器
    default_random_engine eng(10);

    //均匀分布，0-10实数
    uniform_real_distribution<double> rand_xy = uniform_real_distribution<double>(0, 10);

    pcl::PointXYZ pt_random;

    std::vector<std::pair<int, int>> free_lists;

    //x -20~20,y -20~20,随机取点
    for (int i = _x_l; i <= _x_h; i++) {
        for (int j = _y_l; j <= _y_h; j++) {
            //边界点
            if (i == _x_l || i == _x_h || j == _y_l || j == _y_h) {
                pt_random.x = i;
                pt_random.y = j;
                pt_random.z = 0;
                cloudMap.points.push_back(pt_random);
            } else {
                //随机取
                if (rand_xy(eng) < 2) {
                    pt_random.x = i;
                    pt_random.y = j;
                    pt_random.z = 0;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
    }
    
    //无结构的点云，仅1行
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    _has_map = true;

    //PointCloud 转化成PointCloud2
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
}

void RandomMapGenerator::pubSensedPoints() {
    if (!_has_map) return;

    _all_map_pub->publish(globalMap_pcd);
}

int main(int argc, char **argv){

    RCLCPP_INFO(LOGGER, "A* Initialize ~~");

    rclcpp::init(argc, argv);

    auto n = std::make_shared<RandomMapGenerator>();

    // n->RandomMapGenerate();

    // n->pubSensedPoints();

    rclcpp::spin(n);

    rclcpp::shutdown();

    return 0;


}