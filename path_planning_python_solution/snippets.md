### behaviour planning pseudocode

``` python
def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):
    # only consider states which can be reached from current FSM state.
    possible_successor_states = successor_states(current_fsm_state)

    # keep track of the total cost of each state.
    costs = []
    for state in possible_successor_states:
        # generate a rough idea of what trajectory we would
        # follow IF we chose this state.
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)

        # calculate the "cost" associated with that trajectory.
        cost_for_state = 0
        for i in range(len(cost_functions)) :
            # apply each cost function to the generated trajectory
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            # multiply the cost by the associated weight
            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
         costs.append({'state' : state, 'cost' : cost_for_state})

    # Find the minimum cost state.
    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state
``` 

### path planning approaches

#### 1. Check proximity

``` c++
240 vector<double> checkProximity(vector<vector<double> > *sensor_fusion,
241      double car_s, int car_lane) {
242     double best_dist = 999999;
243     double best_bk_dist = 99999;
244     double best_speed = 0;
245     int lane;
246     double dist;
247 
248     for (int i = 0; i < (*sensor_fusion).size(); i++) {
249         lane = getLane((*sensor_fusion)[i][6]);
250         if (lane == -1) continue;
251         // check only cars in the same lane
252         if (car_lane == lane) {
253             dist = (*sensor_fusion)[i][5] - car_s;
254             if (dist >= 0) {
255                 if (best_dist > dist) {
256                     best_dist = dist;
257                     best_speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
258                             (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);
259                 }
260             } else {
261                 dist = abs(dist);
262                 if (best_bk_dist > dist) {
263                     best_bk_dist = dist;
264                 }
265             }
266         }
267     }
268     return {best_dist, best_speed, best_bk_dist};
269 }
```

#### 2. Find best lane

``` c++
182 int findBestLane(vector<vector<double> > *sensor_fusion, double car_s,
183         double car_lane) {
184     vector<double> lanevalues = {0, 0, 0};
185     double dist;
186     int lane;
187     double speed;
188 
189     for (int i = 0; i < (*sensor_fusion).size(); i++) {
190         dist = (*sensor_fusion)[i][5] - car_s;
191         lane = getLane((*sensor_fusion)[i][6]);
192         speed = sqrt((*sensor_fusion)[i][3] * (*sensor_fusion)[i][3] +
193                 (*sensor_fusion)[i][4] * (*sensor_fusion)[i][4]);
194 
195         // cout << "Sensor value: " << (*sensor_fusion)[i][6] << " Sensor lane: " <<
196         // lane << endl;
197 
198         if (lane == -1) continue;
199 
200         if (dist > 0) {
201             lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
202         } else {
203             dist = abs(dist);
204             if (dist <= 10) {
205                 dist = 10;
206                 lanevalues[lane] += 1.0 / (dist * dist) * 1.0 / speed;
207             }
208         }
209     }
```