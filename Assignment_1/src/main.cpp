////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    return u.x() * v.y() - u.y() * v.x();
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    double area_change = det(b - a, d - c);
    if (area_change == 0) return false;

    Vector2d r = c - a, res;

    double t = det(r, d - c) / area_change;
    double u = det(r, b - a) / area_change;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1){
        res = a + t * (b - a);
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    Vector2d edge_point (1e18, query.y());
    int cnt = 0;
    size_t n = poly.size();
    
    for (size_t i = 0; i < n; i++){
        if (intersect_segment(poly[i], poly[(i + 1) % n], query, edge_point)) ++ cnt;
    }
    return cnt %2 == 1;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    int point_cnt = 0;

    if (std::getline(in, line)){
        try{
            point_cnt = std::stoi(line);
        }catch(const std::invalid_argument &e){
            throw std::runtime_error("Invalid Point Count: " + line);
        }catch (const std::out_of_range &e){
            throw std::runtime_error("Point Count Out of Range" + line);
        }
        points.reserve(point_cnt);
        for (int i = 0; i < point_cnt; i ++){
            if (std::getline(in, line)){
                std::istringstream iss(line);
                double x = 0.0, y = 0.0;
                
                if (!(iss >> x >> y)){
                    throw std::runtime_error("Failed to Read Coordinates at Line: " + line); 
                }
                points.emplace_back(x, y);
            } else {
                throw std::runtime_error("Unexpected End of File While Reading Points.");
            }
        }
    } else {
        throw std::runtime_error("Failed to Read Point Count");
    }
    in.close();
    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    std::ofstream outs (filename);
    
    if (!outs.is_open()){
        throw std::runtime_error("Failed to Open File:" + filename);
    }

    outs << points.size() << "\n";
    
    for (const auto &p : points){
        outs << p.x() << ' ' << p.y() << "0\n";
    }

    outs.close();
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{

    Matrix3f A;
    Vector3f b;
    A << 5, 3, -7, -3, 5, 12, 9, -2, -2;
    b << 4, 9, -3;
    std::cout << "Here is the matrix A:\n"
              << A << std::endl;
    std::cout << "Here is the vector b:\n"
              << b << std::endl;
    Vector3f x = A.colPivHouseholderQr().solve(b);
    std::cout << "The solution is:\n"
              << x << std::endl;

    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";

    std::vector<Vector2d> points = load_xyz(points_path);
    std::cout << "inside " << std::endl;
    printf("asf\n");

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d>
        poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }
    save_xyz("output.xyz", result);

    return 0;
}
