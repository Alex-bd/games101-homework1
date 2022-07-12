#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)  //��ͼ�任
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;	//T(view)���󣬿��ƽ����λ���ƶ���ԭ��
    translate <<	1, 0, 0, -eye_pos[0], 
					0, 1, 0, -eye_pos[1],
					0, 0, 1,-eye_pos[2],
					0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)   //ģ�ͱ任����
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

	Eigen::Matrix4f rotation;
	double fangle = rotation_angle / 180 * MY_PI;  //�Ƕȱ仡��

	rotation << cos(fangle), -sin(fangle), 0, 0,
		sin(fangle), cos(fangle), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;		//ģ����ת������z�ᣩ
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
		0, 0, 1, 0;  //͸��ͶӰ���󣬼�ѹ�任����  �����е������Ը���

	double w, h, z;
	h = zNear * tan(eye_fov / 2) * 2;   // t-b���������ײ�
	w = h * aspect_ratio;				//��ȸ��ݸ߶ȼ���
	z = zFar - zNear;					//Զ��ȥ��

	ortho << 2 / w, 0, 0, 0,
			0, 2 / h, 0, 0,
			0, 0, 2 / z, -(zFar + zNear) / 2,
			0, 0, 0, 1; //����ͶӰ������Ϊ�ڹ۲�ͶӰʱx0yƽ���ӽ�Ĭ��Ϊ���ģ��������������ͶӰ�Ͳ���ƽ��x��y�ˣ�

	projection = ortho * proj *projection;


    return projection;
}


Eigen::Matrix4f get_rotation(Vector3f axis, float angle)  //��������ת�����޵����˹��ʽ��Ĭ�Ϲ�ԭ�㣩
{
	double fangle = angle / 180 * MY_PI;	//����
	Eigen::Matrix4f I, N, Rod;
	Eigen::Vector4f axi;
	Eigen::RowVector4f taxi;

	axi << axis.x(), axis.y(), axis.z(), 0;
	taxi << axis.x(), axis.y(), axis.z(), 0;

	I << 1, 0, 0, 0,	//����I����
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	N << 0, -axis.z(), axis.y(), 0,
		axis.z(), 0, -axis.x(), 0,
		-axis.y(), axis.x(), 0, 0,
		0, 0, 0, 1;

	Rod = cos(fangle) * I + (1 - cos(fangle)) * axi * taxi + sin(fangle) * N;
	Rod(3, 3) = 1;//����Ҫע�⣬���������Ĺ�ʽӦ�������������ʱ�ǵ�������ɺ�Ѿ�������½Ǹ�Ϊ1������ᵼ��ͼ�α�������
	std::cout << "Rod: " << Rod << '\n';		//
	return Rod;
}


int main(int argc, const char** argv)
{
    float angle = 0;		//����Ƕ�
    bool command_line = false;		//���������п��ر�־��Ĭ��Ϊ��
    std::string filename = "output.png";


	Eigen::Vector3f raxis(0, 0, 1);
	double rangle = 0, ra;

	//���������
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

	rst::rasterizer r(700, 700);   // �趨700ƽ�����صĹ�դ����ͼ����

    Eigen::Vector3f eye_pos = {0, 0, 5};	//�趨�����ʼλ�ã�Ҫͨ��T��view�����������λ���ƶ���ԭ��

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};	//������λ��

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};		//�趨���������

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);		//������ͼ��Ķ������ţ�������ҵֻһ��

    int key = 0;			//��������
    int frame_count = 0;	//֡���
				
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
	std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//�����޵����˹��ת��ͽ�


    while (key != 27) {			//esc�˳�ѭ��
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		if (rflag) //�������r�ˣ��Ϳ�ʼ�Ƹ�����������ת
			r.set_rodrigues(get_rotation(raxis, rangle));
		else
			r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);			//�ȴ�10������ռ������룬û�������Ϊ�գ�ͼ��������������ԭ״

        std::cout << "frame count: " << frame_count++ << '\n';		//��ʾ��ǰΪ�ڼ�֡

        if (key == 'a') {		//����a����ʱ����ת10��
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
