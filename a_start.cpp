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

// Структура для хранения ребра
struct Edge {
    Point to;
    double weight;

    Edge(const Point& t, double w) : to(t), weight(w) {}
};

// Структура для узла в приоритетной очереди A*
struct AStarNode {
    Point point;
    double f_score; // f = g + h

    AStarNode(const Point& p, double f) : point(p), f_score(f) {}

    // Для priority_queue (минимальная куча)
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

unordered_map<Point, vector<Edge>, PointHash> graph;
unordered_map<Point, double, PointHash> g_score;    // фактическая стоимость от старта
unordered_map<Point, Point, PointHash> came_from;   // для восстановления пути

int total_vertices = 0;
int total_edges = 0;

Point normalize_point(const Point& p) {
    return Point(round(p.lon * 1000000.0) / 1000000.0,
                 round(p.lat * 1000000.0) / 1000000.0);
}

// Простая эвристика (евклидово расстояние в градусах)
double heuristic(const Point& a, const Point& b) {
    double dx = b.lon - a.lon;
    double dy = b.lat - a.lat;
    return sqrt(dx * dx + dy * dy);
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

                    // Добавляем ребро с весом в обе стороны (ненаправленный граф)
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

// Алгоритм A* для поиска пути
pair<vector<Point>, double> a_star(const Point& start, const Point& target) {
    // Проверка наличия вершин
    if (graph.find(start) == graph.end()) {
        cout << "Ошибка: Стартовая точка не найдена в графе!" << endl;
        return {{}, -1};
    }

    if (graph.find(target) == graph.end()) {
        cout << "Ошибка: Конечная точка не найдена в графе!" << endl;
        return {{}, -1};
    }

    // Инициализация
    g_score.clear();
    came_from.clear();

    for (auto& node : graph) {
        g_score[node.first] = numeric_limits<double>::infinity();
    }

    g_score[start] = 0;
    came_from[start] = start;

    // Приоритетная очередь
    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open_set;
    open_set.push(AStarNode(start, heuristic(start, target)));

    int visited_count = 0;

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();
        visited_count++;

        Point current_point = current.point;

        // Если достигли цели
        if (current_point == target) {

            // Восстановление пути
            vector<Point> path;
            Point p = target;
            while (!(p == came_from[p])) {
                path.push_back(p);
                p = came_from[p];
            }
            path.push_back(start);

            vector<Point> reversed_path;
            for (int i = path.size() - 1; i >= 0; i--) {
                reversed_path.push_back(path[i]);
            }

            return {reversed_path, g_score[target]};
        }

        // Перебор соседей
        for (const Edge& edge : graph[current_point]) {
            double tentative_g_score = g_score[current_point] + edge.weight;

            if (tentative_g_score < g_score[edge.to]) {
                // Нашли лучший путь к этому соседу
                came_from[edge.to] = current_point;
                g_score[edge.to] = tentative_g_score;
                double f_score = tentative_g_score + heuristic(edge.to, target);
                open_set.push(AStarNode(edge.to, f_score));
            }
        }
    }

    cout << "Посещено вершин (A*): " << visited_count << endl;
    return {{}, -1}; // Путь не найден
}

int main() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    auto time_start = std::chrono::high_resolution_clock::now(); // Изменено имя переменной

    // Загрузка графа с весами
    load_undirected_graph_with_weights("spb_graph.txt");

    // Заданные координаты
    double start_lat = 59.8667664;
    double start_lon = 30.4692561;
    double target_lat = 59.9269758;
    double target_lon = 30.3374903;

    Point start_point = normalize_point(Point(start_lon, start_lat)); // Изменено имя переменной
    Point target_point = normalize_point(Point(target_lon, target_lat)); // Можно оставить или тоже переименовать

    cout << fixed << setprecision(8);
    cout << "\nКоординаты для поиска:" << endl;
    cout << "Старт: (" << start_point.lon << ", " << start_point.lat << ")" << endl;
    cout << "Цель: (" << target_point.lon << ", " << target_point.lat << ")" << endl;

    // Проверяем наличие точек в графе
    cout << "\nПроверка точек в графе:" << endl;
    cout << "Стартовая точка: " << (graph.find(start_point) != graph.end() ? "Найдена" : "Не найдена") << endl;
    cout << "Конечная точка: " << (graph.find(target_point) != graph.end() ? "Найдена" : "Не найдена") << endl;

    // Запускаем алгоритм A*
    auto result = a_star(start_point, target_point); // Используем новые имена
    vector<Point> path = result.first;
    double total_cost = result.second;

    if (path.empty()) {
        cout << "\nПуть не найден" << endl;
    } else {
        cout << "Общая стоимость пути: " << total_cost << endl;
        cout << "Количество ребер в пути: " << path.size() - 1 << endl;

        auto time_end = std::chrono::high_resolution_clock::now(); // Изменено имя переменной
        std::chrono::duration<double> duration = time_end - time_start; // Вычисляем разницу
        std::cout << "Время выполнения: " << duration.count() << " секунд" << std::endl;
    }
    return 0;
}