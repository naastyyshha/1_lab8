#include <windows.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip>

using namespace std;

struct Point {
    double lon, lat;
    Point(double lng = 0, double lt = 0) : lon(lng), lat(lt) {}
};

// Функция загрузки ВСЕХ точек из файла
vector<Point> load_all_points_from_file(const string& filename) {
    vector<Point> points;
    ifstream file(filename);
    string line;
    
    while (getline(file, line)) {
        if (line.empty()) continue;
        
        size_t colon = line.find(':');
        if (colon == string::npos) continue;
        
        // Первая точка (до ':')
        string first_point_str = line.substr(0, colon);
        size_t comma = first_point_str.find(',');
        double lon1 = stod(first_point_str.substr(0, comma));
        double lat1 = stod(first_point_str.substr(comma + 1));
        points.push_back(Point(lon1, lat1));
        
        // Все остальные точки (после ':')
        string neighbors_part = line.substr(colon + 1);
        stringstream neighbors_stream(neighbors_part);
        string neighbor_str;
        
        while (getline(neighbors_stream, neighbor_str, ';')) {
            if (neighbor_str.empty()) continue;
            
            // Каждая соседняя точка: lon,lat,weight
            stringstream neighbor_ss(neighbor_str);
            string token;
            vector<string> tokens;
            
            while (getline(neighbor_ss, token, ',')) {
                tokens.push_back(token);
            }
            
            if (tokens.size() >= 2) {
                double neighbor_lon = stod(tokens[0]);
                double neighbor_lat = stod(tokens[1]);
                points.push_back(Point(neighbor_lon, neighbor_lat));
            }
        }
    }
    
    cout << "Загружено точек: " << points.size() << endl;
    return points;
}

// Функция поиска ближайшей точки
Point find_closest_point(const vector<Point>& points, double lat, double lon) {
    double min_distance = numeric_limits<double>::max();
    Point closest_point;
    
    for (const auto& point : points) {
        double distance_squared = pow(point.lat - lat, 2) + pow(point.lon - lon, 2);
        
        if (distance_squared < min_distance) {
            closest_point = point;
            min_distance = distance_squared;
        }
    }
    
    return closest_point;
}

int main() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    // Загружаем все точки
    auto points = load_all_points_from_file("spb_graph.txt");
    
    if (points.empty()) {
        cout << "Нет данных для поиска" << endl;
        return 1;
    }
    
    // Ввод координат для поиска
    double search_lat, search_lon;
    cout << "\nВведите координаты для поиска:" << endl;
    cout << "Широта: ";
    cin >> search_lat;
    cout << "Долгота: ";
    cin >> search_lon;
    
    // Поиск ближайшей точки
    Point closest = find_closest_point(points, search_lat, search_lon);
    
    // Вывод результата
    cout << fixed << setprecision(8);
    cout << "\nБлижайшая точка:" << endl;
    cout << "Долгота: " << closest.lon << endl;
    cout << "Широта: " << closest.lat << endl;
    
    return 0;
}