#include <bits/stdc++.h>
#include "json.hpp"

#define all(x) (x).begin(), (x).end()
#define DEBUG
#if defined DEBUG && defined LOCAL
    #define debugdo(x) x
#else
    #define debugdo(x)
#endif

#define debug_arr(x, s) { \
    debug(#x": "); \
    debugdo(for (const auto &_x : x)) \
        debug2(_x, s); \
    debug(""); \
}

#define debug(x) debugdo(cerr << x << endl)
#define debug2(x, s) debugdo(cerr << x << s)
#define debugs(x) debug2(x, ' ')
#define named(x) #x << " = " << x

#ifdef LOCAL
    #define ASSERT(x) assert(x)
#else
    #define ASSERT(x) if (!(x)) { \
        send("error", "("#x")", __LINE__, __FILE__); \
        exit(0); \
    }
#endif

using namespace std;
using nlohmann::json;

template<class T>
string to_string(const T &x)
{
    if constexpr (is_same<char, T>())
        return string(1, x);
    else if constexpr (is_constructible<string, T>())
        return string(x);
    else
        return std::to_string(x);
}

template<class... Args>
inline void send(const string &c, Args... args)
{
	json command;
    command["command"] = c;
    string msg = ((to_string(args) + " ") + ...);
    debug("debug: " << "'" << msg << "'");
    command["debug"] = msg;
    cout << command.dump() << endl;
}

template<>
inline void send(const string &c)
{
    json command;
    command["command"] = c;
    cout << command.dump() << endl;
}

const int N = 31;
array<string, 4> commands{"left", "right", "up", "down"};
array<array<int, 2>, 4> delta{
    array<int, 2>{-1, 0}, array<int, 2>{1, 0},
    array<int, 2>{0, 1}, array<int, 2>{0, -1}
};

unsigned int field[N][N];

inline bool check_mask(unsigned int x, const vector<unsigned int> &v)
{
    for (auto m : v)
    {
        if (m == 0)
        {
            if (x == 0)
                return 1;
            else
                continue;
        }
        if ((m & 7) != (x & 7) && (m & 7) != 0)
            continue;
        if (((m >> 3) & 7) != ((x >> 3) & 7) && ((m >> 3) & 7) != 0)
            continue;
        // if (((m >> 6) & 7) != ((x >> 6) & 7) && ((m >> 6) & 7) != 0)
        //     continue;
        return 1;
    }
    return 0;
}

template<class Path = vector<int>>
int bfs(vector<array<int, 2>> xy,
    const vector<unsigned int> &to,
    const vector<unsigned int> &wall,
    int dep_dir = -1,
    int *first_move = nullptr,
    Path *path = nullptr,
    int *dists = nullptr)
{
    if (path)
        path->clear();
    queue<array<int, 2>> q;
    int d[N][N] = {}, p[N][N] = {};
    for (auto pos : xy)
    {
        q.push(pos);
        p[pos[0]][pos[1]] = -1;
        if (check_mask(field[pos[0]][pos[1]], to))
            return 0;
    }
    int ans = -1;
    while (!q.empty())
    {
        auto [x, y] = q.front();
        q.pop();
        for (int i = 0; i < 4; ++i)
        {
            if (d[x][y] == 0 && dep_dir == i)
                continue;
            auto [dx, dy] = delta[i];
            if (x + dx < 0 || x + dx >= N || y + dy < 0 || y + dy >= N)
                continue;
            if (p[x + dx][y + dy] == 0 &&
                !check_mask(field[x + dx][y + dy], wall))
            {
                if (check_mask(field[x + dx][y + dy], to))
                {
                    if (ans == -1)
                        ans = d[x][y] + 1;
                    if (!dists)
                    {
                        if (path || first_move)
                        {
                            if (path)
                            {
                                path->clear();
                                path->push_back(i);
                            }
                            if (first_move)
                                *first_move = i;
                            while (1)
                            {
                                if (p[x][y] == -1)
                                    break;
                                if (path)
                                    path->push_back(p[x][y] - 1);
                                if (first_move)
                                    *first_move = p[x][y] - 1;
                                int index = p[x][y] - 1;
                                x -= delta[index][0];
                                y -= delta[index][1];
                            }
                            if (path)
                                reverse(all(*path));
                        }
                        return ans;
                    }
                }
                d[x + dx][y + dy] = d[x][y] + 1;
                p[x + dx][y + dy] = i + 1;
                q.push({x + dx, y + dy});
            }
        }
    }
    if (dists)
    {
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (d[i][j] == 0)
                    d[i][j] = INT_MAX;
        for (auto [x, y] : xy)
            d[x][y] = 0;
        copy(d[0], d[0] + N * N, dists);
    }
    return ans;
}

template<class Path = vector<int>>
int bfs(array<int, 2> xy,
    const vector<unsigned int> &to,
    const vector<unsigned int> &wall,
    int dep_dir = -1,
    int *first_move = nullptr,
    Path *path = nullptr,
    int *dists = nullptr)
{
    return bfs(vector{xy}, to, wall, dep_dir, first_move, path, dists);
}

template<class Path = vector<int>>
int bfs(int x, int y,
    const vector<unsigned int> &to,
    const vector<unsigned int> &wall,
    int dep_dir = -1,
    int *first_move = nullptr,
    Path *path = nullptr,
    int *dists = nullptr)
{
    return bfs({x, y}, to, wall, dep_dir, first_move, path, dists);
}

vector<array<unsigned int, N * N>> backups;

void save_field()
{
    backups.push_back(array<unsigned int, N * N>{});
    memcpy(data(backups.back()), field[0], sizeof(field));
}

void load_field()
{
    if (backups.empty())
        return;
    memcpy(field[0], data(backups.back()), sizeof(field));
    backups.pop_back();
}

vector<pair<unsigned int *, unsigned int>> backups_light;

void change_with_save(int x, int y, unsigned int v)
{
    backups_light.push_back({&field[x][y], field[x][y]});
    field[x][y] = v;
}

void load_light()
{
    auto [ptr, v] = backups_light.back();
    backups_light.pop_back();
    *ptr = v;
}

int get_dir_to(int x, int y, unsigned int t)
{
    int cnt[4] = {};
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (field[i][j] == t)
            {
                for (int d = 0; d < 4; ++d)
                {
                    int sp = delta[d][0] * (i - x) + delta[d][1] * (j - y);
                    if (sp > 0)
                        cnt[d]++;
                }
            }
        }
    }
    return max_element(cnt, cnt + 4) - cnt;
}

bool dfs(char used[N][N], int x, int y, unsigned int num, char mode = 1)
{
    if (x < 0 || x >= N || y < 0 || y >= N)
        return 1;
    if ((field[x][y] & 7) == num || used[x][y] == mode)
        return 0;
    used[x][y] = mode;
    bool ans = 0;
    for (auto [dx, dy] : delta)
    {
        if (dfs(used, x + dx, y + dy, num, mode))
            ans = 1;
    }
    return ans;
}

struct Player
{
    int x, y, ticks_per_cell, dir, shift, line_len;
    unsigned num;
    char game_id;
    array<int, 2> get_position() const
    {
        return {x, y};
    }
    bool out_of_bounds()
    {
        return x < 0 || x >= N || y < 0 || y >= N;
    }
};

vector<Player> players;
Player *me = nullptr;

enum Bonus {SAW, NITRO, SLOW};

struct FillRes
{
    int score = 0;
    vector<Player*> killed;
    array<bool, 3> bonuses = {};
    operator double()
    {
        return score;
    }
};

FillRes fill_terr(int x, int y, unsigned int num)
{
    FillRes ans;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if ((field[i][j] >> 3 & 7) == num)
            {
                if ((field[i][j] >> 6 & 7) > 0)
                    ans.bonuses[(field[i][j] >> 6 & 7) - 1] = 1;
                if ((field[i][j] & 7) != num)
                    ans.score += ((field[i][j] & 7) ? 5 : 1);
                change_with_save(i, j, (field[i][j] & ~511) | num);
            }
        }
    }
    char used[N][N] = {};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            if ((field[i][j] & 7) != num && !used[i][j])
                if (dfs(used, i, j, num))
                    dfs(used, i, j, num, 2);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (used[i][j] == 1)
            {
                if ((field[i][j] >> 6 & 7) > 0)
                    ans.bonuses[(field[i][j] >> 6 & 7) - 1] = 1;
                ans.score += ((field[i][j] & 7) ? 5 : 1);
                change_with_save(i, j, (field[i][j] & ~455) | num);
            }
        }
    }
    for (auto &p : players)
        if (used[p.x][p.y])
            ans.killed.push_back(&p);
    return ans;
}

void add(array<int, 2> &a, const array<int, 2> &b)
{
    a[0] += b[0];
    a[1] += b[1]; 
}

inline pair<bool, int> turn(Player &p1, Player &p2, int depth)
{
    if (p1.shift == 0)
    {
        auto [dx, dy] = delta[p1.dir];
        p1.x += dx;
        p1.y += dy;
        p1.shift = p1.ticks_per_cell;
    }
    if (p2.shift == 0)
    {
        auto [dx, dy] = delta[p2.dir];
        p2.x += dx;
        p2.y += dy;
        p2.shift = p2.ticks_per_cell;
    }
    if (p1.out_of_bounds() || (field[p1.x][p1.y] >> 3 & 7) == p1.num)
        return {1, -1e5};
    if (p2.out_of_bounds() || (field[p2.x][p2.y] >> 3 & 7) == p2.num)
        return {1, 1e5};
    auto new_pos1 = p1.get_position();
    auto new_pos2 = p2.get_position();
    auto old_pos1 = new_pos1;
    auto old_pos2 = new_pos2;
    add(old_pos1, delta[p1.dir ^ 1]);
    add(old_pos2, delta[p2.dir ^ 1]);

    if (new_pos1 == new_pos2 ||
        (old_pos1 == new_pos2 && p1.dir != p2.dir) ||
        (old_pos2 == new_pos1 && p1.dir != p2.dir))
    {
        if (p1.line_len >= p2.line_len)
            return {1, -1000 * depth};
        else
            return {1, 1000 * depth};
    }

    int mn = min(p1.shift, p2.shift);
    p1.shift -= mn;
    p2.shift -= mn;
    int ans = 0;
    bool f12 = 0, f21 = 0;
    if (!p1.shift)
    {
        if (((field[p1.x][p1.y] >> 3) & 7) == p2.num)
            f12 = 1;
        if ((field[p1.x][p1.y] & 7) != p1.num)
        {
            p1.line_len++;
            change_with_save(p1.x, p1.y, (field[p1.x][p1.y] & ~(7 << 3)) | (p1.num << 3));
        }
        else if (p1.line_len)
        {
            p1.line_len = 0;
            auto [x1, y1] = old_pos2;
            auto [x2, y2] = new_pos2;
            bool in_terr = ((field[x2][y2] & 7) == p1.num);
            ans += fill_terr(p1.x, p1.y, p1.num);
            if ((field[x1][y1] & 7) == p1.num && (field[x2][y2] & 7) == p1.num && !in_terr)
                return {1, 1000 * depth};
        }
    }
    if (!p2.shift)
    {
        if (((field[p2.x][p2.y] >> 3) & 7) == p1.num)
            f21 = 1;
        if ((field[p2.x][p2.y] & 7) != p2.num)
        {
            p2.line_len++;
            change_with_save(p2.x, p2.y, (field[p2.x][p2.y] & ~(7 << 3)) | (p2.num << 3));
        }
        else if (p2.line_len)
        {
            p2.line_len = 0;
            auto [x1, y1] = old_pos1;
            auto [x2, y2] = new_pos1;
            bool in_terr = ((field[x2][y2] & 7) == p2.num);
            ans -= fill_terr(p2.x, p2.y, p2.num);
            if ((field[x1][y1] & 7) == p2.num && (field[x2][y2] & 7) == p2.num && !in_terr)
                return {1, -1000 * depth};
        }
    }
    if (f21 && p1.line_len)
        return {1, -1000 * depth};
    if (f12 && p2.line_len)
        return {1, 1000 * depth};
    return {0, ans};
}

pair<int, int> minimax(const Player &p1, const Player &p2,
    int depth = 8,
    int alpha = INT_MIN,
    int beta = INT_MAX,
    bool second = 0)
{
    ASSERT(p1.shift == 0 || p2.shift == 0);
    if (depth < 0 && !second)
        return {0, 0};
    bool maxi;
    if (p1.shift == 0 && p2.shift == 0)
        maxi = !second;
    else
        maxi = (p1.shift == 0);
    int best, ans = 0;
    if (maxi)
        best = INT_MIN;
    else
        best = INT_MAX;
    auto sz = backups_light.size();
    for (int i = 0; i < 4; ++i)
    {
        if ((maxi && i == (p1.dir ^ 1)) || (!maxi && i == (p2.dir ^ 1)))
            continue;

        Player np1 = p1, np2 = p2;

        if (maxi)
            np1.dir = i;
        else
            np2.dir = i;

        int res = 0;

        if (maxi && p2.shift == 0)
            res = minimax(np1, np2, depth - 1, alpha, beta, 1).first;
        else
        {
            auto [f, r] = turn(np1, np2, depth + 1);
            res = r;
            if (!f)
                res += minimax(np1, np2, depth - 1, alpha, beta, 0).first;
        }
        
        while (backups_light.size() > sz)
            load_light();
        if ((maxi && res > best) || (!maxi && res < best))
        {
            ans = i;
            best = res;
            if (maxi)
                alpha = max(alpha, res);
            else
                beta = min(beta, res);
            if (alpha > beta)
                return {best, ans};
        }
    }
    return {best, ans};
}

const double LEN_COEF = 0;
const double ENEMY_DIST_COEF = 0;
const double NEAR_CELL_COEF = 0;
const double NEAR_CELL_BASE = 0.8;
const double ENEMY_CELL_BONUS = 0;
const double ENEMY_CELL_DIST_COEF = -4;
const double KILL_BONUS = 500;
array<pair<int, double>, 3> dist_coef = {pair(5, 1), pair(10, 0.5), pair(30, 0.05)};

pair<int, double> find_path_with_oreder(int my_x, int my_y, int my_dir, int ticks,
    const array<int, 4> &dir_order)
{
    save_field();
    auto sz = backups_light.size();
    char used[N][N] = {};
    vector<array<int, 2>> candidates;
    for (auto &p : players)
    {
        if (p.num == 1)
            continue;
        auto [x, y] = p.get_position();
        used[x][y] = 1;
        for (auto j : dir_order)
        {
            if ((j ^ p.dir) == 1)
                continue;
            auto [dx, dy] = delta[j];
            candidates.push_back({x + dx, y + dy});
        }
    }
    sort(all(candidates));
    candidates.erase(unique(all(candidates)), candidates.end());
    int n[4][N][N][N];
    memset(n, -1, sizeof(n));
    int dp[4][N][N][N];
    memset(dp, -1, sizeof(dp));
    double best = -INFINITY;
    int best_move = -1;
    array<bool, 7> ret = {};
    ret[1] = 1;
    function<bool(int, int, int, int)> calc_dp = [&](int dd, int x, int y, int l)
    {
        if (used[x][y])
            return false;
        if ((field[x][y] & 7) == 1 && l == 0)
            return true;
        if (l == 0)
            return false;
        if (dp[dd][x][y][l] != -1)
            return (bool)dp[dd][x][y][l];
        change_with_save(x, y, field[x][y] | 8);
        double ans = -INFINITY;
        dp[dd][x][y][l] = 0;
        for (auto i : dir_order)
        {
            if (i == dd)
                continue;
            auto [dx, dy] = delta[i];
            if (x + dx < 0 || x + dx >= N || y + dy < 0 || y + dy >= N ||
                ((field[x + dx][y + dy] >> 3) & 7) == 1 || used[x + dx][y + dy])
                continue;
            if (calc_dp(i ^ 1, x + dx, y + dy, l - 1))
            {
                int xx = x + dx, yy = y + dy, ll = l - 1, ddd = i ^ 1;
                auto sz = backups_light.size();
                bool f = 0;
                while (ll > 0)
                {
                    if (used[xx][yy] || n[ddd][xx][yy][ll] == -1)
                    {
                        f = 1;
                        break;
                    }
                    if ((field[xx][yy] & 7) != 1)
                        change_with_save(xx, yy, field[xx][yy] | 8);
                    auto [ddx, ddy] = delta[n[ddd][xx][yy][ll]];
                    ddd = n[ddd][xx][yy][ll] ^ 1;
                    xx += ddx;
                    yy += ddy;
                    ll--;
                }
                if (f)
                    continue;
                dp[dd][x][y][l] = 1;

                auto fr = fill_terr(xx, yy, 1);
                double res = fr.score;

                if (res > 1e-8)
                    res /= l + me->line_len + bfs(xx, yy, {2, 3, 4, 5, 6}, {}, ddd);
                else
                    res = -bfs(xx, yy, {2, 3, 4, 5, 6}, {}, ddd);

                if (fr.bonuses[SAW])
                    res += 1;
                if (fr.bonuses[NITRO])
                    res += 1;
                if (fr.bonuses[SLOW])
                    res -= 1;

                // for (auto &p : players)
                // {
                //     if (ret[p.num])
                //         continue;
                //     bool f = 0;
                //     for (int x = 0; x < N && !f; ++x)
                //     {
                //         for (int y = 0; y < N && !f; ++y)
                //         {
                //             if ((field[x][y] >> 3 & 7) == p.num)
                //             {
                //                 for (auto [dx, dy] : delta)
                //                 {
                //                     if (x + dx >= 0 && x + dx < N &&
                //                         y + dy >= 0 && y + dy < N &&
                //                         (field[x + dx][y + dy] & 7) == p.num)
                //                     {
                //                         f = 1;
                //                         break;
                //                     }
                //                 }
                //             }
                //         }
                //     }
                //     if (!f && (field[p.x][p.y] & 7) != p.num)
                //         res += KILL_BONUS;
                // }

                int min_dist = INT_MAX;
                for (auto &p : players)
                    if (p.num != 1)
                        min_dist = min(min_dist, abs(xx - p.x) + abs(yy - p.y));
                res += min_dist * ENEMY_DIST_COEF;
                // res += l * LEN_COEF;
                // bool ec = 0;
                // for (int dx = -2; dx <= 2 && !ec; ++dx)
                // {
                //     for (int dy = -2; dy <= 2 && !ec; ++dy)
                //     {
                //         if (xx + dx >= 0 && xx + dx < N && yy + dy >= 0 && yy + dy < N &&
                //             (field[x + dx][y + dy] & 7) >= 2)
                //         {
                //             ec = 1;
                //             res += ENEMY_CELL_BONUS;
                //         }
                //     }
                // }
                min_dist = INT_MAX;

                // for (int x = 0; x < N; ++x)
                // {
                //     for (int y = 0; y < N; ++y)
                //     {
                //         if ((field[x][y] & 7) < 2)
                //             continue;
                //         int dist = abs(x - xx) + abs(y - yy);
                //         for (auto [d, c] : dist_coef)
                //         {
                //             if (dist <= d)
                //             {
                //                 res += c;
                //                 break;
                //             }
                //         }
                //     }
                // }

                //res += min_dist * ENEMY_CELL_DIST_COEF;

                while (backups_light.size() > sz)
                    load_light();
                if (res > ans)
                {
                    ans = res;
                    n[dd][x][y][l] = i;
                }
            }
        }
        if (dp[dd][x][y][l] && x == my_x && y == my_y && best < ans)
        {
            debug(named(l) << " " << named(ans));
            best = ans;
            best_move = n[dd][x][y][l];
        }
        load_light();
        return (bool)dp[dd][x][y][l];
    };
    int enemy_dist[5][N][N], territory_dist[5][N][N];
    for (auto &p : players)
    {
        if (p.num == 1)
            continue;
        unsigned i = p.num - 2;
        bfs(p.get_position(), {}, {}, p.dir ^ 1,
            nullptr, (vector<int>*)nullptr, (int*)enemy_dist[i]);
        vector<array<int, 2>> terr;
        for (int x = 0; x < N; ++x)
            for (int y = 0; y < N; ++y)
                if ((field[x][y] & 7) == i + 2)
                    terr.push_back({x, y});
        bfs(terr, {}, {}, -1, nullptr, (vector<int>*)nullptr, (int*)territory_dist[i]);
    }
    int my_dist = bfs(my_x, my_y, {1}, {8}, my_dir ^ 1);
    for (int l = 1; l < min(N, ticks); ++l)
    {
        bool f = 0;
        // vector<array<int, 2>> new_candidates;
        // for (auto [x, y] : candidates)
        // {
        //     if (x < 0 || x >= N || y < 0 || y >= N || used[x][y])
        //         continue;
        //     if (((field[x][y] >> 3) & 7) == 1)
        //     {
        //         f = 1;
        //         break;
        //     }
        //     used[x][y] = 1;
        //     for (int j = 0; j < 4; ++j)
        //     {
        //         auto [dx, dy] = delta[j];
        //         new_candidates.push_back({x + dx, y + dy});
        //     }
        // }

        for (int x = 0; x < N && !f; ++x)
        {
            for (int y = 0; y < N && !f; ++y)
            {
                used[x][y] = 0;
                for (auto &p : players)
                {
                    if (p.num == 1)
                        continue;
                    int i = p.num - 2;
                    // int dist = INT_MAX;
                    // for (auto &o : players)
                    // {
                    //     if (o.num == 1)
                    //         continue;
                    //     if (o.num != p.num)
                    //         dist = min(dist, enemy_dist[o.num - 2][x][y]);
                    // }
                    if (enemy_dist[i][x][y] <= l)// &&
                        //enemy_dist[i][x][y] + territory_dist[i][x][y] <= dist)
                    {
                        if (((field[x][y] >> 3) & 7) == 1)
                            f = 1;
                        if ((field[x][y] & 7) == p.num && enemy_dist[i][x][y] == l)
                        {
                            change_with_save(x, y, 7);
                            vector<int> path;
                            bfs(p.get_position(), {7}, {p.num << 3}, p.dir ^ 1,
                                nullptr, &path);
                            load_light();
                            auto [xx, yy] = p.get_position();
                            for (auto d : path)
                            {
                                auto [dx, dy] = delta[d];
                                xx += dx;
                                yy += dy;
                                change_with_save(xx, yy, field[xx][yy] | p.num << 3);
                            }
                            fill_terr(xx, yy, p.num);
                        }
                        used[x][y] = 1;
                        break;
                    }
                }
            }
        }
        if (f)
            break;
        for (auto &p : players)
            if (p.num > 1 && territory_dist[p.num - 2][p.x][p.y] <= l)
                ret[p.num] = 1;
        // candidates.swap(new_candidates);
        calc_dp(my_dir ^ 1, my_x, my_y, l);
    }
    backups_light.erase(backups_light.begin() + sz, backups_light.end());
    load_field();
    return {best_move, best};
}

pair<int, double> find_path(int my_x, int my_y, int my_dir, int ticks)
{
    array<int, 4> indx;
    iota(all(indx), 0);
    static mt19937 rnd(42);
    int mv = -1;
    double best = -INFINITY;
    for  (int i = 0; i < 3; ++i)
    {
        shuffle(all(indx), rnd);
        auto [nmv, res] = find_path_with_oreder(my_x, my_y, my_dir, ticks, indx);
        if (res > best)
        {
            best = res;
            mv = nmv;
        }
    }
    return {mv, best};
}

int find_kill()
{
    int enemy_dist[5][N][N], territory_dist[5][N][N];
    array<int, 5> max_dist = {};
    for (auto &p : players)
    {
        if (p.num == 1)
            continue;
        auto i = p.num - 2;
        bfs(p.get_position(), {}, {(i + 2) << 3}, p.dir ^ 1,
            nullptr, (vector<int>*)nullptr, (int*)enemy_dist[i]);
        max_dist[i] = INT_MAX;
        vector<array<int, 2>> terr;
        for (int x = 0; x < N; ++x)
            for (int y = 0; y < N; ++y)
                if ((field[x][y] & 7) == p.num)
                {
                    max_dist[i] = min(max_dist[i], enemy_dist[i][x][y]);
                    terr.push_back({x, y});
                }
        bfs(terr, {}, {}, -1, nullptr, (vector<int>*)nullptr, (int*)territory_dist[i]);
    }
    bool used[N][N] = {};
    for (auto &p : players)
    {
        if (p.num == 1)
            continue;
        int i = p.num - 2;
        for (int l1 = 0; l1 < min(max_dist[i], N); ++l1)
        {
            for (int l2 = 0; l2 < N; ++l2)
            {
                save_field();
                for (int x = 0; x < N; ++x)
                {
                    for (int y = 0; y < N; ++y)
                    {
                        used[x][y] = 0;
                        if ((field[x][y] & 7) == 1)
                            continue;
                        for (auto &o : players)
                        {
                            if (o.num == 1)
                                continue;
                            if ((p.num == o.num && enemy_dist[o.num - 2][x][y] <= l1) ||
                                enemy_dist[o.num - 2][x][y] <= l1 + l2)
                            {
                                used[x][y] = 1;
                                break;
                            }
                        }
                        if (used[x][y])
                            field[x][y] |= 7;
                    }
                }
                vector<int> path;
                int fm = -1;
                int dist = bfs(me->get_position(), {p.num << 3},
                    {7, 8}, me->dir ^ 1, &fm, &path);
                ASSERT((fm == -1) == (dist == -1));
                if (dist == -1 || dist >= l1)
                {
                    load_field();
                    continue;
                }
                auto [x, y] = me->get_position();
                for (auto i : path)
                {
                    x += delta[i][0];
                    y += delta[i][1];
                    field[x][y] |= 8;
                }
                int dist2 = bfs(x, y, {1}, {7, 8},
                    (path.empty() ? me->dir : path.back()) ^ 1);
                load_field();
                if (dist2 == -1)
                {
                    load_field();
                    continue;
                }
                dist += dist2;
                if (dist < l1 + l2)
                    return fm;
            }
        }
    }
    return -1;
}

inline bool is_my(unsigned int x)
{
    return (x & 7) == 1 || (x & 0b111000) == 8;
}

long start_time;
stringstream debug_buf;

template<class... Args>
void send_with_info(const string &c, Args... args)
{
    string coords;
    for (auto &p : players)
        coords += "{"s + p.game_id + ":" + to_string(p.x) + ":" + to_string(p.y) + "}";
    send(c, args..., "time =", (clock() - start_time) / (double)CLOCKS_PER_SEC, "s",
        coords, debug_buf.str());
    debug_buf.str("");
}

const int MAX_TICKS = 2500;

int main()
{
    debugdo(ofstream log("log.txt"));
    srand(time(0));
    string input_string;
    getline(cin, input_string);
    debugdo(log << input_string << endl);
    int WIDTH = json::parse(input_string)["params"]["width"];
    deque<int> path_to_space;
    map<string, int> command_to_index;
    map<string, Bonus> str_to_bonus{{"s", SLOW}, {"n", NITRO}, {"saw", SAW}};
    for (int i = 0; i < 4; ++i)
        command_to_index[commands[i]] = i;
    players.reserve(7);
    start_time = clock();
    while (true)
    {
    	memset(field, 0, sizeof(field));
        getline(cin, input_string);
        if (input_string.empty())
            continue;
        debugdo(log << input_string << endl);
        json state = json::parse(input_string);
        if (state["type"] == "end_game")
            break;
        int tick = state["params"]["tick_num"];
        auto &jp = state["params"]["players"];
        players.clear();
        me = nullptr;
        int pn = 2;
        ASSERT(state["params"]["bonuses"].is_array());
        for (auto b : state["bonuses"])
        {
            int x = b["position"][0].get<int>() / WIDTH;
            int y = b["position"][1].get<int>() / WIDTH;
            field[x][y] |= (str_to_bonus.at(b["type"]) + 1) << 6;
        }
        for (auto it = jp.begin(); it != jp.end(); ++it)
        {
            auto &p = *it;
            players.push_back(Player());
            auto &cur = players.back();
            cur.game_id = it.key()[0];
            if (cur.game_id == 'i')
            {
                cur.num = 1;
                me = &cur;
            }
            else
                cur.num = pn++;
            if (p["direction"].is_null())
                cur.dir = -1;
            else
                cur.dir = command_to_index[p["direction"]];
            bool nitro = 0, slow = 0;
            for (auto &b : p["bonuses"])
                if (b["type"] == "n")
                    nitro = 1;
                else if (b["type"] == "s")
                    slow = 1;
            int speed = 5;
            if (nitro && !slow)
                speed = 6;
            else if (slow && !nitro)
                speed = 3;
            cur.ticks_per_cell = WIDTH / speed;
            int x = p["position"][0].get<int>();
            int y = p["position"][1].get<int>();
            cur.x = x / WIDTH;
            cur.y = y / WIDTH;
            cur.shift = 0;
            if (x % WIDTH != WIDTH / 2)
            {
                if (cur.dir == 0 && x % WIDTH < WIDTH / 2)
                    cur.x--;
                else if (cur.dir == 1 && x % WIDTH > WIDTH / 2)
                    cur.x++;
                cur.shift = (x - WIDTH / 2) % WIDTH / speed;
                if (cur.dir == 1)
                    cur.shift = (cur.ticks_per_cell - cur.shift) % cur.ticks_per_cell;
            }
            else if (y % WIDTH != WIDTH / 2)
            {
                if (cur.dir == 3 && y % WIDTH < WIDTH / 2)
                    cur.y--;
                else if (cur.dir == 2 && x % WIDTH > WIDTH / 2)
                    cur.y++;
                cur.shift = (y - WIDTH / 2) % WIDTH / speed;
                if (cur.dir == 2)
                    cur.shift = (cur.ticks_per_cell - cur.shift) % cur.ticks_per_cell;
            }
            unsigned c = cur.num;
            for (auto &coord : p["territory"])
            {
                int x = coord[0].get<int>() / WIDTH;
                int y = coord[1].get<int>() / WIDTH;
                field[x][y] |= c;
            }
            c <<= 3;
            cur.line_len = p["lines"].size();
            for (auto &coord : p["lines"])
            {
                int x = coord[0].get<int>() / WIDTH;
                int y = coord[1].get<int>() / WIDTH;
                field[x][y] |= c;
            }
        }
        ASSERT(me);
        ASSERT(me->num == 1);

        bool in_home = ((field[me->x][me->y] & 7) == 1);

        if (!in_home)
            path_to_space.clear();

        int fme = find_kill();
        int fmh = -1;

        if (fme == -1)
        {
            int dist = INT_MAX;
            Player *enemy = nullptr;
            bool mm = 0;
            for (auto &p : players)
            {
                auto [x, y] = p.get_position();
                if (p.game_id == 'i')
                    continue;
                if (abs(x - me->x) + abs(y - me->y) < dist)
                {
                    enemy = &p;
                    dist = abs(x - me->x) + abs(y - me->y);
                }
            }
            if (dist <= 3)
            {
                ASSERT(enemy);
                auto [res, mv] = minimax(*me, *enemy);
                mm = 1;
                send_with_info(commands[mv], "minimax", enemy->game_id, res);
            }
            if (mm)
            {
                continue;
            }

            if (me->dir == -1)
                me->dir = 1;
            if (players.size() > 1)
            {
                auto [fm, res] = find_path(me->x, me->y, me->dir, (MAX_TICKS - tick) / 6);
                debug("find_path " << res);
                fmh = fm;
                debug_buf << named(res);
            }
            if (fmh == -1)
            {
                bfs(me->x, me->y, {1}, {8}, me->dir ^ 1, &fmh);
            }
        }

        bool keep_dir = 0;
        if (players.size() == 1 && tick < 1300)
        {
            for (int i = 0; i < N; ++i)
                if (!is_my(field[0][i]) || !is_my(field[N - 1][i]) ||
                    !is_my(field[i][0]) || !is_my(field[i][N - 1]))
                {
                    keep_dir = 1;
                    break;
                }
        }
        
        // int fmd = -1;
        // if (in_home)
        // {
        //     for (auto [x, y] : enemy_pos)
        //     {
        //         if ((field[x][y] & 7) == 1)
        //         {
        //             field[x][y] |= 7;
        //             bfs(me->x, me->y, {7}, {8}, me->dir ^ 1, &fmd);
        //             field[x][y] &= ~7;
        //             field[x][y] |= 1;
        //         }
        //         if (fmd != -1)
        //         {
        //             try_keep_dir = 0;
        //             break;
        //         }
        //     }
        // }
        
        // if (try_keep_dir && fme == -1 && !in_home &&
        //     (me->x != 0 || me->dir != 0) && (me->x != N - 1 || me->dir != 1) &&
        //     (me->y != 0 || me->dir != 3) && (me->y != N - 1 || me->dir != 2))
        // {
        //     save_field();
        //     int x = me->x;
        //     int y = me->y;
        //     x += delta[me->dir][0];
        //     y += delta[me->dir][1];
        //     vector<int> path;
        //     int my_dist;
        //     if (((field[x][y] >> 3) & 7) == 1)
        //         my_dist = -1;
        //     else
        //         my_dist = bfs(x, y, {1}, {8}, me->dir ^ 1, nullptr, &path);
        //     if (my_dist != -1)
        //         my_dist *= ticks_for_cell[1];
        //     for (auto i : path)
        //     {
        //         field[x][y] |= 8;
        //         x += delta[i][0];
        //         y += delta[i][1];
        //     }
        //     int enemy_dist = INT_MAX;
        //     for (unsigned int i = 0; i < enemy_pos.size(); ++i)
        //     {
        //         int dist = bfs(enemy_pos[i], {8}, {(i + 2) << 3}, enemy_dir[i] ^ 1);
        //         if (dist != -1)
        //             dist *= ticks_for_cell[i + 2];
        //         if (dist != -1 && dist < enemy_dist)
        //             enemy_dist = dist;
        //     }
        //     if ((enemy_dist > my_dist + 2 || enemy_dist == -1) && my_dist != -1)
        //         keep_dir = 1;
        //     load_field();
        // }

        if (fme != -1)
        {
            if ((fme ^ me->dir) == 1)
                fme = me->dir;
            send_with_info(commands[fme], "kill");
        }
        else if (keep_dir)
        {
            if ((me->x == 0 && me->dir == 0) ||
                (me->x == N - 1 && me->dir == 1))
                me->dir = 2 + (me->y != 0);
            if ((me->y == 0 && me->dir == 3) ||
                (me->y == N - 1 && me->dir == 2))
                me->dir = (me->x == 0);
            send_with_info(commands[me->dir], "keep");
        }
        else
        {
            // if (in_home && path_to_space.empty())
            // {
            //     if (players.size() == 1 || true)
            //         bfs(me->x, me->y, {0, 2, 3, 4, 5, 6}, {}, me->dir ^ 1,
            //             nullptr, &path_to_space);
            //     else
            //     {
            //         double best = -INFINITY;
            //         int tx = -1, ty = -1;
            //         for (int x = 0; x < N; ++x)
            //         {
            //             for (int y = 0; y < N; ++y)
            //             {
            //                 if ((field[x][y] & 7) == 1)
            //                     continue;
            //                 int dist = abs(x - me->x) + abs(y - me->y);
            //                 for (int i = 0; i < 4; ++i)
            //                 {
            //                     auto [dx, dy] = delta[i];
            //                     if (x + dx < 0 || x + dx >= N ||
            //                         y + dy < 0 || y + dy >= N)
            //                         continue;
            //                     if ((field[x + dx][y + dy] & 7) == 1)
            //                     {
            //                         change_with_save(x, y, 64);
            //                         auto res = find_path(x, y, i ^ 1,
            //                             (MAX_TICKS - tick) / 6 - dist).second;
            //                         // res -= LEN_PENALTY * dist;
            //                         if (res > best)
            //                         {
            //                             best = res;
            //                             tx = x;
            //                             ty = y;
            //                         }
            //                         load_light();
            //                         break;
            //                     }
            //                 }
            //             }
            //         }
            //         change_with_save(tx, ty, 7);
            //         bfs(me->x, me->y, {7}, {}, me->dir ^ 1, &fmh);
            //             //nullptr, &path_to_space);
            //         load_light();
            //     }
            // }
            // if (in_home && !path_to_space.empty())
            // {
            //     fmh = path_to_space.front();
            //     path_to_space.pop_front();
            // }
            if ((me->x == 0 && fmh == 0) ||
                (me->x == N - 1 && fmh == 1))
                fmh = 2 + (me->y != 0);
            if ((me->y == 0 && fmh == 3) ||
                (me->y == N - 1 && fmh == 2))
                fmh = (me->x == 0);
            string path;
            for (auto i : path_to_space)
                path += commands[i][0];
            send_with_info(commands[fmh], "home", path);
        }
    }
}