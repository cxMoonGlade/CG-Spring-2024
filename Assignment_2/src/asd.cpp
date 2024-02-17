// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.); 
                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}


Vector3d parallelogram_matrix(Vector3d ray_origin, Vector3d ray_direction, Vector3d pgram_origin, Vector3d pgram_u, Vector3d pgram_v){
    // ray_orgin + t*ray_direction =  pgram_origin + u*pgram_u + v*pgram_v

    Vector3d d;
    d << pgram_origin - ray_origin;

    Matrix3d A;
    A << -pgram_u, -pgram_v, ray_direction;

    Vector3d t = A.colPivHouseholderQr().solve(d);

    return t;
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // TODO: Check if the ray intersects with the parallelogram
            Vector3d t = parallelogram_matrix(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v);
            // u: t[0], v: t[1], t: t[2]
            if ((t[0]>0 && t[1]>0 && t[2]>0 && t[0]<1 && t[1]<1))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + (t[2]* ray_direction);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);




    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - ray_origin).normalized();

            // TODO: Check if the ray intersects with the parallelogram
            Vector3d t = parallelogram_matrix(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v);
            if ((t[0]>0 && t[1]>0 && t[2]>0 && t[0]<1 && t[1]<1))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                Vector3d ray_intersection = ray_origin + (t[2]* ray_direction);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

// TODO: implement the generic ray sphere intersection
bool Ray_intersects_sphere(Vector3d ray_origin, Vector3d ray_direction, Vector3d sphere_center, double sphere_radius){
    // We need to find the solutions of f(p(t)) = 0 to find the intersection
    // Using the quadratic formula to slove t
    double A = ray_direction.dot(ray_direction);
    double B = ray_direction.dot(ray_origin-sphere_center)*2;
    double C = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);

    //Find the discriminant
    double disc = (B*B) - (4*A*C);
    double sqrt_disc = sqrt((B*B) - (4*A*C));

    if(disc >=0){
        //Check if t is positive or negative(no solution)
        double plus_t = (-B + sqrt_disc)/2*A;
        double mins_t = (-B - sqrt_disc)/2*A;
        return ((plus_t >0) || (mins_t > 0));
    }else{
        return false; // no intersection
    }
}

Vector3d sphere_intersection_point(Vector3d ray_origin, Vector3d ray_direction, Vector3d sphere_center, double sphere_radius){
    double A = ray_direction.dot(ray_direction);
    double B = ray_direction.dot(ray_origin-sphere_center)*2;
    double C = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);

    double disc = (B*B) - (4*A*C);
    double sqrt_disc = sqrt((B*B) - (4*A*C));

    double plus_t = (-B + sqrt_disc)/2*A;
    double mins_t = (-B - sqrt_disc)/2*A;

    double t;

    if(plus_t> 0 && mins_t >0){
        if(plus_t == mins_t){
            t = plus_t;
        }else{
            t = fmin(plus_t,mins_t);
        }
    }else if(plus_t>0 && mins_t<0){
        t = plus_t;
    }else if(plus_t<0 && mins_t>0){
        t = mins_t;
    }
    // return Intersection point
    return ray_origin + (t * ray_direction);

}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800); 
    MatrixXd G = MatrixXd::Zero(800, 800); 
    MatrixXd B = MatrixXd::Zero(800, 800); 
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - ray_origin).normalized();

            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection
            if (Ray_intersects_sphere(ray_origin,ray_direction, sphere_center, sphere_radius))
            {
                // TODO: The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = sphere_intersection_point(ray_origin,ray_direction, sphere_center, sphere_radius);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // TODO: Add shading parameter here
                // Shading Variables
                Vector3d light_direction = (light_position - ray_intersection).normalized();
                Vector3d view_direction = (camera_origin - ray_intersection).normalized();
                Vector3d h = (light_direction + view_direction).normalized();

                Vector3d diffuse = diffuse_color * fmax(0, ray_normal.dot(light_direction));
                Vector3d specular = specular_color * std::pow(fmax(0, ray_normal.dot(h)), specular_exponent);
                // const double diffuse = (light_position - ray_intersection).normalized().dot(ray_normal);
                // const double specular = (light_position - ray_intersection).normalized().dot(ray_normal);



                // Simple diffuse model
                R(i, j) = ambient + diffuse[0] + specular[0];
                R(i, j) = std::max(R(i, j), 0.0);

                G(i, j) = ambient + diffuse[1] + specular[1];
                G(i, j) = std::max(G(i, j), 0.0);

                B(i, j) = ambient + diffuse[2] + specular[2];
                B(i, j) = std::max(B(i, j), 0.0);

                // // Clamp to zero
                // C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
