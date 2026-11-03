#include"functions.hpp"
double pi = M_PI;
std::mutex mtx;
std::mutex human_mutex;
std::condition_variable cv;
std::mt19937 gen(std::random_device{}());
std::uniform_real_distribution<double> angle_dist(0.0, 2*pi);
std::uniform_int_distribution<int> dist(0, 99);
State state;

double human_velocity= 1.25; // 1.25 m/s human velocity
double robot_velocity= 5.0; // 5 m/s robot velocity
double theta = 0;
double hx, hy;

int iterations =-1;
int parent[1];

bool flag = false;
bool allow = true;
bool right_move = false;
bool left_move = false;
bool up_move = false;
bool down_move = false;

vector<vector<pair<int,double>>> changed_path;
std::vector<std::tuple<int, int, double>> path;
HumanState human_state = HumanState::StandingCorridor;

vector<vector<double>> transitionMatrix = {

   // From StandingCorridor
   {0.10, 0.20, 0.15, 0.15, 0.10, 0.10, 0.05, 0.05, 0.05, 0.05},

   // From StandingNearDesk
   {0.15, 0.10, 0.20, 0.10, 0.10, 0.10, 0.05, 0.05, 0.10, 0.05},

   // From Exercising
   {0.10, 0.20, 0.10, 0.15, 0.10, 0.10, 0.05, 0.05, 0.10, 0.05},

   // From WaitingElevator
   {0.20, 0.10, 0.10, 0.10, 0.15, 0.10, 0.05, 0.05, 0.10, 0.05},

   // From ReadingBoard
   {0.15, 0.10, 0.10, 0.15, 0.10, 0.10, 0.10, 0.05, 0.10, 0.05},

   // From TalkingPhone
   {0.10, 0.15, 0.10, 0.10, 0.10, 0.15, 0.10, 0.05, 0.10, 0.05},

   // From Tinkering
   {0.10, 0.10, 0.10, 0.10, 0.10, 0.15, 0.15, 0.05, 0.10, 0.05},

   // From StandingKitchen
   {0.10, 0.10, 0.05, 0.10, 0.10, 0.15, 0.10, 0.15, 0.10, 0.05},

   // From RestingLobby
   {0.15, 0.10, 0.10, 0.10, 0.10, 0.10, 0.05, 0.05, 0.20, 0.05},

   // From CleaningShelves
   {0.20, 0.10, 0.10, 0.15, 0.10, 0.10, 0.05, 0.05, 0.10, 0.05}
};

vector<StateInfo> states = {
   {HumanState::StandingCorridor, 0},
   {HumanState::StandingNearDesk, 1},
   {HumanState::Exercising, 2},
   {HumanState::WaitingElevator, 3},
   {HumanState::ReadingBoard, 4},
   {HumanState::TalkingPhone, 5},
   {HumanState::Tinkering, 6},
   {HumanState::StandingKitchen, 7},
   {HumanState::RestingLobby, 8},
   {HumanState::CleaningShelves, 9}
};

HumanState nextState(HumanState current) {

   int currentIndex = static_cast<int>(current);

   discrete_distribution<> dist(
      transitionMatrix[currentIndex].begin(),
      transitionMatrix[currentIndex].end()
   );

   int nextIndex = dist(gen);

   return states[nextIndex].state;
}

void humanThread(State& state){
   bool change = true;

   while (!flag) {

      double x, y;
      double timer=0;
      // 1. READ shared state
      {
         lock_guard<mutex> lock(human_mutex);
         x = state.human_pos.first;
         y = state.human_pos.second;
      }

      double new_x, new_y;
      bool moving= (rand() % 100 >= 30);

      if(moving){
         // 2. compute movement
         if(change == true){
            theta = angle_dist(gen);
            change = false;
         }

         new_x = x + human_velocity * std::cos(theta);
         new_y = y + human_velocity * std::sin(theta);

         new_x = std::max(0.0, std::min(990.0, new_x));
         new_y = std::max(0.0, std::min(990.0, new_y));

         // 3. simulate movement delay (NO lock)
         std::this_thread::sleep_for(
         std::chrono::milliseconds(1000));

         // 4. WRITE shared state
         {
         std::lock_guard<std::mutex> lock(human_mutex);

         state.human_pos = {new_x, new_y};
         }
         std::cout << "Human: ("
                     << x << "m," << y << "m) -> ("
                     << new_x << "m," << new_y << "m)\n";
      }else{
         //activities he does while not moving
         //choose a human_state from human markov chain states,with switch, (cases)
         {
         std::lock_guard<std::mutex> lock(human_mutex);
         human_velocity = 0;
         }
         cout<<"human is working...\n";
         HumanState current = human_state;
         int value = static_cast<int>(current);
         
         switch (value) {

            case 0:
               timer = 20;    // 20s
               break;
            case 1:
               timer = 30;    // 30s
               break;
            case 2:
               timer = 40;    // 40s
               break;
            case 3:
               timer = 50;    // 50s
               break;
            case 4:
               timer = 60;    // 60s
               break;
            case 5:
               timer = 70;    // 70s
               break;
            case 6:
               timer = 80;    // 80s
               break;
            case 7:
               timer = 90;    // 90s
               break;
            case 8:
               timer = 100;   // 100s
               break;
            case 9://worst case
               timer = 200;   // 200s
               break;
         }
         std::this_thread::sleep_for(
         std::chrono::duration<double>(timer)
         );
         human_state = nextState(current);
         {
         lock_guard<mutex> lock(human_mutex);
         human_velocity = 1.25;
         }
         change = true;
      }
   }
}

void robotThread(State& state, int goal, vector<tuple<int, int, double>> path){
   cout << "path size = " << path.size() << endl;
   for (auto& [u, v, w] : path) {
      int stay;
      do{
         stay=0;
         //0. Wait the monnitor
         {
         lock_guard<mutex> lock(mtx);
         state.monitoring = true;
         }
         cv.notify_one();

         cout<<"Robot waits for response\n";
         // Terminate for reasons
         if(state.terminate == true){
            break;
         }
         // 1. WAIT conditions (OUTSIDE mutex)
         unique_lock<mutex> lock(mtx);
         cv.wait(lock, [&state]{return state.robot_moving;});
         state.robot_moving = false;
         lock.unlock();

         cout<<"Robot got permission\n";
         // Save previous node
         if(!state.go_back){
            parent[0] = {u};
         }

         double time =0;
         int ux = u % 100;
         int uy = u / 100;
         ux =double(ux) * w;
         uy = double(uy) * w;

         int vx = v % 100;
         int vy = v / 100;
         vx = double(vx) * w;
         vy = double(vy) * w;

         // 2.Slow mode on
         if(state.slow_down_mode == true){
            robot_velocity = 3.5;
            time = w / robot_velocity;

            std::this_thread::sleep_for(
            std::chrono::duration<double>(time));
            std::cout << "Robot: ("
            << ux << "m," << uy << "m) -> ("
            << vx << "m," << vy << "m), the distance is " << w << "m\n";

            robot_velocity = 5.0;
         }

         // 4. Terminate conditions
         if(v == goal){
            state.terminate= true;
            time = w / robot_velocity;

            std::this_thread::sleep_for(
            std::chrono::duration<double>(time));

            std::cout << "Robot: ("
                  << ux << "m," << uy << "m) -> ("
                  << vx << "m," << vy << "m), the distance is " << w << "m\n";

            state.robot_moving = false;
            break;
         }

         // 5. Manuevering mode on
         double t_wait=0;
         if(state.manuevering == true){
            time = w / robot_velocity;

            if((uy > vy)  || (vy > uy)){
               t_wait = std::abs(uy - hy) / human_velocity;

               if(right_move == true){
                  right_move = false;
                  vx = ux + w;
               }
               else if(left_move == true){
                  left_move = false;
                  vx = ux - w;
               }
               std::this_thread::sleep_for(
               std::chrono::duration<double>(time));

               std::cout << "Robot: ("
               << ux << "m," << uy << "m) -> ("
               << vx << "m," << vy << "m), the distance is " << w << "m\n";

               std::this_thread::sleep_for(
               std::chrono::duration<double>(t_wait));

               stay = 1;

               std::this_thread::sleep_for(
               std::chrono::duration<double>(time));

               std::cout << "Robot: ("
               << vx << "m," << vy << "m) -> ("
               << ux << "m," << uy << "m), the distance is " << w << "m\n";
            }
            else if( (ux > vx)  || (vx > ux)){
               t_wait = std::abs(ux - hx) / human_velocity;

               if(up_move == true){
                  up_move = false;
                  vy = uy - w;
               }
               else if(down_move == true){
                  down_move = false;
                  vy = uy + w;
               }

               std::this_thread::sleep_for(
               std::chrono::duration<double>(time));

               std::cout << "Robot: ("
               << ux << "m," << uy << "m) -> ("
               << vx << "m," << vy << "m), the distance is " << w << "m\n";

               std::this_thread::sleep_for(
               std::chrono::duration<double>(t_wait));

               stay = 1;

               std::this_thread::sleep_for(
               std::chrono::duration<double>(time));

               std::cout << "Robot: ("
               << vx << "m," << vy << "m) -> ("
               << ux << "m," << uy << "m), the distance is " << w << "m\n";
            }
         }

         // 6.Wait mode is on
         if(state.wait){
            cout<<"Robot needs to wait because human is in the way\n";
            stay = 1;
            double t_wait = 4;
            std::this_thread::sleep_for(
            std::chrono::duration<double>(t_wait));
         }

         // 7.Go back mode on
         if(state.go_back){
            time = w / robot_velocity;
            cout<< "Robot needs to go back to estimate a new path.\n";
            vx = parent[0] % 100;
            vy = parent[0] / 100;

            std::this_thread::sleep_for(
            std::chrono::duration<double>(time));

            std::cout << "Robot: ("
               << ux << "m," << uy << "m) -> ("
               << vx << "m," << vy << "m), the distance is " << w << "m\n";

            state.terminate = true;
         }

         // 8. UPDATE share data
         if(!state.manuevering && !state.slow_down_mode && !state.go_back && !state.wait){
            // Normal mode on
            time = w / robot_velocity;
            std::this_thread::sleep_for(
            std::chrono::duration<double>(time));

            std::cout << "Robot: ("
            << ux << "m," << uy << "m) -> ("
            << vx << "m," << vy << "m), the distance is " << w << "m\n";
         }
         //8.5 Wait to secure value
         {
            unique_lock<mutex> lock(mtx);
            cv.wait(lock, [&state]{return state.estimation_completed;});
            state.estimation_completed=false;
            lock.unlock();
         }
         // 9. UPDATE flags
         state.wait= false;
         state.manuevering = false;
         state.slow_down_mode = false;
         state.go_back = false;

         // 10. SYNCHRONIZE
      }while(stay == 1);
   }
   state.terminate= true;
   cout<<"job is done\n";
}

void monitorThread(State& state,vector<tuple<int, int, double>> path, vector<vector<pair<int,double>>> adj, vector<tuple<int,int,double>> edges, vector<bool> blocked, int n, int m, int goal)
{
   for(auto& [u, v, w] : path)
   {
      int worst_case = 0; 
      int counter= 1;
      int stop_signal = 0;
      double human_speed;
      double cos, sin;

      if(state.terminate == true){
         break;
      }

      do{
            cout<<"Monitor waits for response\n";
            unique_lock<mutex> lock(human_mutex);

            cv.wait(lock, [&state]{return state.monitoring;});
            state.monitoring = false;
            lock.unlock();

            stop_signal = 0;

            if(state.terminate == true){
               break;
            }

            cout<<"Monitor is watching\n";
            {
            lock_guard<mutex> lock(human_mutex);
            hx = state.human_pos.first;
            hy = state.human_pos.second;
            }

            int vx = v % n;
            int vy = v / n;
            vx = double(vx) * w;
            vy = double(vy) * w;

            int ux = u % n;
            int uy = u / n;
            ux = double(ux) * w;
            uy = double(uy) * w;

            double dx = ux - hx;
            double dy = uy - hy;

            double  tolerance  = 0.2; // tolerance in seconds
            double  eps = 2.5; // epsilon in meters

            double sign;
            double S;
            double t_cross;
            {
            lock_guard<mutex> lock(human_mutex);
            cos = std::cos(theta);
            sin = std::sin(theta);
            human_speed = human_velocity;
            }

            if((uy > vy)  || (vy > uy)){  // robot is moving up or down
               sign = dx * cos;
               double y_cross;
               double y_min;
               double y_max;
               double t_r;
               if(human_speed == 0){
                  if(dx == 0){
                     if((hy < uy && hy > vy) || (hy > uy && hy < vy)){
                        blocked[v]=true;
                        worst_case = 1;
                        cout<<"Human stays still in the way, robot must change path.\n";
                     }else{
                        cout<<"Robot can pass freely\n";
                        state.robot_moving = true;
                     }
                  }else{
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
               }
               else if(sign <0){
                  cout<< "Robot can pass freely\n";
                  state.robot_moving = true;
               }
               else if(sign >0){
                  S = dx / cos;
                  t_cross = S / human_velocity;
                  y_cross = hy + S*sin;
                  y_min = std::min(uy, vy);
                  y_max = std::max(uy, vy);

                  if(y_cross >= y_min && y_cross <= y_max){
                     t_r= abs(uy - y_cross) / robot_velocity;
                     if(abs(t_cross - t_r) <= tolerance){
                        cout<<"Robot is in danger of collision, change to slow mode\n";
                        state.slow_down_mode = true;
                        state.robot_moving = true;
                     }
                     else{
                        cout<<"Robot can pass freely\n";
                        state.robot_moving = true;
                     }
                  }
               }
               else{
                  double t_collision;
                  if((ux == hx) && (cos!= 0)){
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
                  else if((ux != hx) && (cos == 0)){
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
                  else if((ux == hx) && (cos == 0)){
                     if((hy > uy && uy <vy) && sin == -1){

                        t_collision = (hy-uy) / (robot_velocity + human_velocity);
                        y_cross = uy + robot_velocity * t_collision;

                        if((y_cross <= vy) || (y_cross >= vy && (y_cross - vy <= eps))){
                           // possible colission move, check if robot can go left or right.
                           if(blocked[u+1] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              right_move = true;
                           }
                           else if(blocked[u-1] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              left_move = true;
                           }
                           else{
                              worst_case = 1;
                              cout<<"Robot is in danger of collision, go back\n";
                              state.go_back = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              blocked[u]= true;
                           }
                        }else{
                           cout<<"Robot can pass freely\n";
                           state.robot_moving = true;
                        }
                     }
                     else if((hy > uy && hy<vy && uy <vy) && sin == 1){
                        // they go in the same direction, no collision
                        if(counter == 2){
                           worst_case = 1;
                           blocked[v] = true;
                        }else{
                           counter++;
                           state.wait = true;
                           state.robot_moving = true;
                           stop_signal = 1;
                        }
                     }
                     else if((hy < uy && uy > vy) && sin == 1){

                        t_collision = (uy-hy) / (robot_velocity + human_velocity);
                        y_cross = uy - robot_velocity * t_collision;

                        if((y_cross >= vy) || (y_cross <= vy && (vy - y_cross <= eps))){
                           // possible colission move, check if robot can go left or right.
                           if(blocked[u+1] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              right_move = true;
                           }
                           else if(blocked[u-1] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal =1;
                              left_move = true;
                           }
                           else{
                              worst_case = 1;
                              cout<<"Robot is in danger of collision, go back\n";
                              state.go_back = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              blocked[u] = true;
                           }
                        }else{
                           cout<<"Robot can pass freely\n";
                           state.robot_moving = true;
                        }
                     }
                     else if((hy < uy &&  hy> vy && uy > vy) && sin == -1){
                        // they go in the same direction, no collision
                        if(counter == 2){
                           worst_case = 1;
                           blocked[v]=true;
                        }else{
                           counter++;
                           state.wait = true;
                           state.robot_moving = true;
                           stop_signal = 1;
                        }
                     }
                  }
               }
            }
            else if((ux > vx)  || (vx > ux)){  // robot is moving left or right
               sign = dy * sin;
               double x_cross;
               double x_min;
               double x_max;
               double t_r;
               if(human_speed  == 0){
                  if(dy == 0){
                     if((hx < ux && hx > vx) || (hx > ux && hx < vx)){
                        blocked[v]=true;
                        worst_case = 1;
                        cout<<"Human stays still in the way, robot must change path.\n";
                     }else{
                        cout<<"Robot can pass freely\n";
                        state.robot_moving = true;
                     }
                  }else{
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
               }
               else if(sign < 0){
                  cout<< "Robot can pass freely\n";
                  state.robot_moving = true;
               }
               else if(sign > 0){
                  S = dy / sin;
                  t_cross = S / human_velocity;
                  x_cross = hx + S*cos;
                  x_min = std::min(ux, vx);
                  x_max = std::max(ux, vx);

                  if(x_cross >= x_min && x_cross <= x_max){
                     t_r= abs(ux - x_cross) / robot_velocity;
                     if(abs(t_cross - t_r) <= tolerance){
                        cout<<"Robot is in danger of collision, change to slow mode\n";
                        state.slow_down_mode = true;
                     }
                     else{
                        cout<<"Robot can pass freely\n";
                        state.robot_moving = true;
                     }
                  }
               }
               else{
                  double t_collision;
                  if((uy == hy) && (sin!= 0)){
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
                  else if((uy != hy) && (sin == 0)){
                     cout<<"Robot can pass freely\n";
                     state.robot_moving = true;
                  }
                  else if((uy == hy) && (sin == 0)){
                     if((hx > ux && ux <vx) && cos == -1){

                        t_collision = (hx-ux) / (robot_velocity + human_velocity);
                        x_cross = ux + robot_velocity * t_collision;

                        if((x_cross <= vx) || (x_cross >= vx && (x_cross - vx <= eps))){
                              // possible colission move
                           if(blocked[u - n] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              up_move = true;
                           }
                           else if(blocked[u + n] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              down_move = true;
                           }
                           else{
                              worst_case = 1;
                              cout<<"Robot is in danger of collision, go back\n";
                              state.go_back = true;
                              state.robot_moving = true;
                              stop_signal = 1;
                              blocked[u]= true;
                           }
                        }else{
                           cout<<"Robot can pass freely\n";
                           state.robot_moving = true;
                        }
                     }
                     else if((hx > ux  &&  hx < vx && ux <vx) && cos == 1){
                        // they go in the same direction, no collision
                        if(counter == 2){
                           worst_case =1;
                           blocked[v]= true;
                        }else{
                           counter++;
                           state.wait = true;
                           state.robot_moving =true;
                           stop_signal =1;
                        }
                     }
                     else if((hx < ux && ux > vx) && cos == 1){

                        t_collision = (ux-hx) / (robot_velocity + human_velocity);
                        x_cross = ux - robot_velocity * t_collision;

                        if((x_cross >= vx) || (x_cross <= vx && (vx - x_cross <= eps))){
                           // possible colission move
                           if(blocked[u - n] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal =1;
                              up_move = true;
                           }
                           else if(blocked[u + n] == false){
                              state.manuevering = true;
                              state.robot_moving = true;
                              stop_signal =1;
                              down_move = true;
                           }
                           else{
                              worst_case = 1;
                              cout<<"Robot is in danger of collision, go back\n";
                              state.go_back = true;
                              state.robot_moving = true;
                              stop_signal =1;
                              blocked[u] = true;
                           }
                        }else{
                           cout<<"Robot can pass freely\n";
                           state.robot_moving = true;
                        }
                     }
                     else if((hx > vx && hx < ux && ux > vx) && cos == -1){
                        // they go in the same direction, no collision
                        if(counter == 2){
                           worst_case=1;
                           blocked[v] = true;
                        }else{
                           counter++;
                           state.wait = true;
                           state.robot_moving = true;
                           stop_signal =1;
                        }
                     }
                  }
               }
            }                     
            if(worst_case == 1){
               // estimate Dijkstra's algorithm to find a new path
               cout<<"Worst case happens, robot needs to replan the path.\n";
               path.clear();
               int N = m* n;
               vector<vector<pair<int,double>>> adj_new(N);
               for(size_t u =0; u< adj.size(); u++){
                  if (blocked[u]) continue;

                  for(auto& [v, w]: adj[u]){
                     if(!blocked[v]){
                        adj_new[u].push_back({v, w});
                     }
                  }
               }
               int start = parent[0];
               vector<tuple<int, int, double>> OE = OE_subgraph(N, start, goal, adj_new, edges);

               vector<tuple<int, int, double>> path = BFS(OE, N, start, goal);
               if(path.empty()){
                  allow = false;
               }
               blocked[u] = false;
               blocked[v] = false;
               state.terminate = true;
               iterations++;
            }
         {
         lock_guard<mutex> lock(mtx);
         state.estimation_completed = true;
         state.robot_moving = true;
         }
         cv.notify_one();
      }while(stop_signal == 1);
   }
}

int human_robot_motion(vector<tuple<int,int,double>> edges_min, vector<vector<pair<int,double>>> adj_min, vector<tuple<int,int,double>> edges, vector<vector<pair<int,double>>> adj, vector<bool> blocked, int n, int m, vector<int> points){
   //Pre-processing
   int N = m*n;
   int x, y;
   int j = points[0] % n;
   int i = points[0] / n;

   do {
      x = dist(gen);
      y = dist(gen);
   } while (x == j  && y == i);
   j= j * 10;
   i= i * 10;
   state.human_pos = {double(x)*10, double(y)*10};

   cout<<"Simulation starts\n";
   cout<<"Human starts from ("<<state.human_pos.first<<"m,"<<state.human_pos.second<<"m)"
   <<" and the Robot from ("<<j<<"m,"<<i<<"m)\n";

   std::thread t1(humanThread,std::ref(state));

   for(size_t i = 0; i < points.size() - 1; i++){
      int root = points[i];
      int goal = points[i + 1];
      vector<tuple<int,int,double>> OE_min = OE_min_subgraph(N, root, goal, adj_min, edges_min);

      //filter OG to use to sp problem with original weights
      map<pair<int,int>, double> originalWeight;
      vector<tuple<int,int,double>> OE_min_updated;
      vector<vector<pair<int,double>>> adj_plus(N);
      // Store original weights
      for (const auto& [u, v, w] : edges) {
         originalWeight[{u, v}] = w;
         originalWeight[{v, u}] = w;
      }

      set<tuple<int,int,double>> seen;
      for (const auto& e : OE_min) {
         if (!seen.insert(e).second)
            continue;

         auto [u, v, dummy] = e;
         double w = originalWeight.at({u,v});
         
         adj_plus[u].push_back({v,w});
         adj_plus[v].push_back({u,w});

         OE_min_updated.emplace_back(u,v,w);
      }
      vector<tuple<int,int,double>> OE = OE_subgraph(N, root, goal, adj_plus, OE_min_updated);
      vector<tuple<int, int, double>> path = BFS(OE, N, root, goal);

      if(path.empty()){
         cout<<"there is no path among some of the vertices.\n";
         break;
      }
      //Simulation Robot movement
      std::thread t2(robotThread,std::ref(state), goal, path);
      std::thread t3(monitorThread, std::ref(state), path, adj, edges, blocked, n, m, goal);
      t2.join();
      t3.join();
      for(int i=0; i<=iterations; i++){
         if(allow){
            std::thread t2(robotThread,std::ref(state), goal, path);
            std::thread t3(monitorThread, std::ref(state), path, adj, edges, blocked, n, m, goal);
            t2.join();
            t3.join();
            state.terminate = false;
         }else{
            cout<<"The robot stuck somewhere on the path!\n";
        }
      }
      state.terminate = false;
   }
   flag = true;
   t1.join();
   return 0;
}

