#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotating;
    float angle = rotation_angle/180*MY_PI;

    rotating<< std::cos(angle),-1*std::sin(angle),0,0,\
               std::sin(angle),std::cos(angle),0,0,\
               0,0,1,0,\
               0,0,0,1;
    
    model=rotating*model;

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

    Eigen::Matrix4f orthographic_matrix = Eigen::Matrix4f::Identity();    
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f presp_to_ortho_matrix = Eigen::Matrix4f::Identity();
    double top = std::tan((eye_fov/2)/180*MY_PI/2)*zNear;
    double right = top*aspect_ratio;
    float bottom = -top;
    float left = -right;
 
    scale<< 2/(right-left),0,0,0,\
            0,2/(top-bottom),0,0,\
            0,0,2/(zNear-zFar),0,\
            0,0,0,1;
    translate<< 1,0,0,-1*(right+left)/2,\
                0,1,0,-1*(top+bottom)/2,\
                0,0,1,-1*(zNear+zFar)/2,\
                0,0,0,1;

    orthographic_matrix = scale*translate;

    presp_to_ortho_matrix<< zNear,0,0,0,\
                            0,zNear,0,0,\
                            0,0,zNear+zFar,0,\
                            0,0,1,0;
    projection = orthographic_matrix*presp_to_ortho_matrix;
                        
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis,float angle){
    Eigen::Matrix4f rotating = Eigen::Matrix4f::Identity();
    float radian = angle/180*MY_PI;
    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    float cos_angle = std::cos(radian);
    float sin_angle = std::sin(radian);

    rotating << x*x+(1-x*x)*cos_angle,x*y*(1-cos_angle)+z*sin_angle,x*z*(1-cos_angle)-y*sin_angle,0,\
                x*y*(1-cos_angle)-z*sin_angle,y*y+(1-y*y)*cos_angle,y*z*(1-cos_angle)+sin_angle,0,\
                x*z*(1-cos_angle)+y*sin_angle,y*z*(1-cos_angle)-x*sin_angle,z*z+(1-z*z)*cos_angle,0,\
                0,0,0,1;
                
    return rotating;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 10};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

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

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
