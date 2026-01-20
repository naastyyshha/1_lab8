#include <iostream>
#include <windows.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <iomanip>
#include <limits>
#include <chrono>

using namespace std;

struct Point {
    double lon, lat;
    
    Point(double lng = 0, double lt = 0) : lon(lng), lat(lt) {}
    
    bool operator==(const Point& p) const {
        const double epsilon = 0.000001;
        return fabs(lon - p.lon) < epsilon && fabs(lat - p.lat) < epsilon;
    }
};

struct PointHash {
    size_t operator()(const Point& p) const {
        return hash<double>()(p.lon) ^ (hash<double>()(p.lat) << 1);
    }
};

// Структура для хранения ребра с весом
struct Edge {
    Point to;
    double weight;
    
    Edge(const Point& t, double w) : to(t), weight(w) {}
};

// Структура для приоритетной очереди в алгоритме Дейкстры
struct QueueNode {
    Point point;
    double distance;
    
    QueueNode(const Point& p, double d) : point(p), distance(d) {}
    
    // минимальная куча
    bool operator>(const QueueNode& other) const {
        return distance > other.distance;
    }
};

unordered_map<Point, vector<Edge>, PointHash> graph;
unordered_map<Point, double, PointHash> dist;
unordered_map<Point, Point, PointHash> parent;

int total_vertices = 0;
int total_edges = 0;

Point normalize_point(const Point& p) {
    return Point(round(p.lon * 1000000.0) / 1000000.0,
                 round(p.lat * 1000000.0) / 1000000.0);
}

void load_undirected_graph_with_weights(const string& filename) {
    ifstream file(filename);
    string line;
    
    while (getline(file, line)) {
        if (line.empty()) continue;
        
        size_t colon = line.find(':');
        if (colon == string::npos) continue;
        
        // Исходная вершина
        string from_str = line.substr(0, colon);
        size_t comma = from_str.find(',');
        
        double lon1 = stod(from_str.substr(0, comma));
        double lat1 = stod(from_str.substr(comma + 1));
        Point from = normalize_point(Point(lon1, lat1));
        
        // Добавляем вершину если её нет
        if (graph.find(from) == graph.end()) {
            graph[from] = {};
            total_vertices++;
        }
        
        // Все соседи с весами
        stringstream ss(line.substr(colon + 1));
        string neighbor;
        
        while (getline(ss, neighbor, ';')) {
            if (neighbor.empty()) continue;
            
            size_t first_comma = neighbor.find(',');
            if (first_comma != string::npos) {
                size_t second_comma = neighbor.find(',', first_comma + 1);
                
                if (second_comma != string::npos) {
                    // Читаем координаты и вес
                    double lon2 = stod(neighbor.substr(0, first_comma));
                    double lat2 = stod(neighbor.substr(first_comma + 1, second_comma - first_comma - 1));
                    double weight = stod(neighbor.substr(second_comma + 1));
                    
                    Point to = normalize_point(Point(lon2, lat2));
                    
                    // Добавляем вершину-сосед если её нет
                    if (graph.find(to) == graph.end()) {
                        graph[to] = {};
                        total_vertices++;
                    }
                    
                    // Добавляем ребро с весом в обе стороны
                    graph[from].push_back(Edge(to, weight));
                    graph[to].push_back(Edge(from, weight));
                    total_edges += 2; // Учитываем оба направления
                }
            }
        }
    }
    
    cout << "Загружено вершин: " << total_vertices << endl;
    cout << "Загружено ребер с весами: " << total_edges << endl;
}

// Алгоритм Дейкстры для поиска кратчайшего пути по весу
pair<vector<Point>, double> dijkstra(const Point& start, const Point& target) {
    // Проверка наличия вершин
    if (graph.find(start) == graph.end()) {
        cout << "Ошибка: Стартовая точка не найдена в графе" << endl;
        return {{}, -1};
    }
    
    if (graph.find(target) == graph.end()) {
        cout << "Ошибка: Конечная точка не найдена в графе" << endl;
        return {{}, -1};
    }
    
    // Инициализация расстояний
    for (auto& node : graph) {
        dist[node.first] = numeric_limits<double>::infinity();
    }
    
    dist[start] = 0;
    parent[start] = start;
    
    // Приоритетная очередь (минимальная куча)
    priority_queue<QueueNode, vector<QueueNode>, greater<QueueNode>> pq;
    pq.push(QueueNode(start, 0));
    
    int visited_count = 0;
    
    while (!pq.empty()) {
        QueueNode current = pq.top();
        pq.pop();
        visited_count++;
        
        Point cur_point = current.point;
        double cur_dist = current.distance;
        
        // Если нашли более короткий путь к этой вершине, пропускаем
        if (cur_dist > dist[cur_point]) {
            continue;
        }
        
        // Если достигли цели
        if (cur_point == target) {
            
            // Восстановление пути
            vector<Point> path;
            Point p = target;
            while (!(p == parent[p])) {
                path.push_back(p);
                p = parent[p];
            }
            path.push_back(start);
            
            vector<Point> reversed_path;
            for (int i = path.size() - 1; i >= 0; i--) {
                reversed_path.push_back(path[i]);
            }
            
            return {reversed_path, dist[target]};
        }
        
        // Перебор соседей с учетом весов ребер
        for (const Edge& edge : graph[cur_point]) {
            double new_dist = dist[cur_point] + edge.weight;
            
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                parent[edge.to] = cur_point;
                pq.push(QueueNode(edge.to, new_dist));
            }
        }
    }
    
    cout << "Посещено вершин: " << visited_count << endl;
    return {{}, -1}; // Путь не найден
}

int main() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    auto start_time = std::chrono::high_resolution_clock::now();
    // Загрузка графа с весами
    load_undirected_graph_with_weights("spb_graph.txt");
    
    // Заданные координаты
    double start_lat = 59.8667664;
    double start_lon = 30.4692561;
    double target_lat = 59.9269758;
    double target_lon = 30.3374903;
    
    Point start = normalize_point(Point(start_lon, start_lat));
    Point target = normalize_point(Point(target_lon, target_lat));
    
    cout << fixed << setprecision(8);
    cout << "\nКоординаты для поиска:" << endl;
    cout << "Старт: (" << start.lon << ", " << start.lat << ")" << endl;
    cout << "Цель: (" << target.lon << ", " << target.lat << ")" << endl;
    
    // Проверяем наличие точек в графе
    cout << "\nПроверка точек в графе:" << endl;
    cout << "Стартовая точка: " << (graph.find(start) != graph.end() ? "Найдена" : "Не найдена") << endl;
    cout << "Конечная точка: " << (graph.find(target) != graph.end() ? "Найдена" : "Не найдена") << endl;
    
    // Запускаем алгоритм Дейкстры
    auto result = dijkstra(start, target);
    vector<Point> path = result.first;
    double total_weight = result.second;
    
    if (path.empty()) {
        cout << "\nПуть не найден" << endl;
        
        // Выводим информацию о достижимости
        int reachable_count = 0;
        for (const auto& d : dist) {
            if (d.second < numeric_limits<double>::infinity()) {
                reachable_count++;
            }
        }
        cout << "Из стартовой точки достижимо: " << reachable_count << " вершин" << endl;
        cout << "Всего вершин в графе: " << total_vertices << endl;
        
        if (reachable_count < total_vertices) {
            cout << "Граф несвязный" << endl;
        }
    } else {
        cout << "Общий вес пути: " << total_weight << endl;
        cout << "Количество ребер в пути: " << path.size() - 1 << endl;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> duration = end_time - start_time;
std::cout << "Время выполнения: " << duration.count() << " секунд" << std::endl;
    return 0;
}