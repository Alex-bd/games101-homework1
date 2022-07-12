#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)  //视图变换
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;	//T(view)矩阵，控制将相机位置移动到原点
    translate <<	1, 0, 0, -eye_pos[0], 
					0, 1, 0, -eye_pos[1],
					0, 0, 1,-eye_pos[2],
					0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)   //模型变换矩阵
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

	Eigen::Matrix4f rotation;
	double fangle = rotation_angle / 180 * MY_PI;  //角度变弧度

	rotation << cos(fangle), -sin(fangle), 0, 0,
		sin(fangle), cos(fangle), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;		//模型旋转矩阵（绕z轴）
	model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
	Eigen::Matrix4f proj, ortho;

	proj << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;  //透视投影矩阵，挤压变换矩阵，  第三行的求解相对复杂

	double w, h, z;
	h = zNear * tan(eye_fov / 2) * 2;   // t-b：顶部减底部
	w = h * aspect_ratio;				//宽度根据高度计算
	z = zFar - zNear;					//远减去近

	ortho << 2 / w, 0, 0, 0,
			0, 2 / h, 0, 0,
			0, 0, 2 / z, -(zFar + zNear) / 2,
			0, 0, 0, 1; //正交投影矩阵，因为在观测投影时x0y平面视角默认为中心，所以这里的正交投影就不用平移x和y了；

	projection = ortho * proj *projection;


    return projection;
}


Eigen::Matrix4f get_rotation(Vector3f axis, float angle)  //任意轴旋转矩阵（罗德里格斯公式，默认过原点）
{
	double fangle = angle / 180 * MY_PI;	//弧度
	Eigen::Matrix4f I, N, Rod;
	Eigen::Vector4f axi;
	Eigen::RowVector4f taxi;

	axi << axis.x(), axis.y(), axis.z(), 0;
	taxi << axis.x(), axis.y(), axis.z(), 0;

	I << 1, 0, 0, 0,	//定义I矩阵
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	N << 0, -axis.z(), axis.y(), 0,
		axis.z(), 0, -axis.x(), 0,
		-axis.y(), axis.x(), 0, 0,
		0, 0, 0, 1;

	Rod = cos(fangle) * I + (1 - cos(fangle)) * axi * taxi + sin(fangle) * N;
	Rod(3, 3) = 1;//这里要注意，非齐次坐标的公式应用在齐次坐标上时记得运算完成后把矩阵的右下角改为1，否则会导致图形比例错误
	std::cout << "Rod: " << Rod << '\n';		//
	return Rod;
}


int main(int argc, const char** argv)
{
    float angle = 0;		//定义角度
    bool command_line = false;		//定义命令行开关标志，默认为关
    std::string filename = "output.png";


	Eigen::Vector3f raxis(0, 0, 1);
	double rangle = 0, ra;

	//命令行相关
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

	rst::rasterizer r(700, 700);   // 设定700平方像素的光栅器视图窗口

    Eigen::Vector3f eye_pos = {0, 0, 5};	//设定相机初始位置，要通过T（view）矩阵将相机的位置移动到原点

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};	//三顶点位置

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};		//设定三顶点序号

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);		//保存多个图像的顶点和序号，本次作业只一个

    int key = 0;			//键盘输入
    int frame_count = 0;	//帧序号
				
    if (command_line) {			
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

	bool rflag = false;
	std::cout << "Please enter the axis and angle:" << std::endl;
	std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//定义罗德里格斯旋转轴和角


    while (key != 27) {			//esc退出循环
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		if (rflag) //如果按下r了，就开始绕给定任意轴旋转
			r.set_rodrigues(get_rotation(raxis, rangle));
		else
			r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);			//等待10号码接收键盘输入，没有输入就为空，图像不做调整，保持原状

        std::cout << "frame count: " << frame_count++ << '\n';		//显示当前为第几帧

        if (key == 'a') {		//按下a，逆时针旋转10度
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
		else if (key == 'r') {
			rflag = true;
			rangle += ra;
		}
    }

    return 0;
}
