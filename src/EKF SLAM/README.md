# EKF SLAM known correspondences(µt−1, Σt−1, ut, zt)
<p align="justify">

SLAM address one of the most fundamental problems in robotics,
the simultaneous localization and mapping problem. 
This problem is commonly abbreviated as SLAM, and is also known as Concurrent Mapping and Localization, or CML.
SLAM problems arise when the robot does not have access to a map of the environment nor does it have access to its own poses.
Instead, all it is given are measurements z1:t and controls u1:t.
The term “simultaneous localization and mapping” describes the resulting problem.
In SLAM, the robot acquires a map of its environment while simultaneously localizing itself relative to this map.
From a probabilistic perspective, there are two main forms of the SLAM problem, which are both of equal practical importance.
One is known as the online SLAM problem and another is known as full slam problem.</p>

## Online SLAM problem:
<p align="justify">

It involves estimating the posterior over the momentary pose along with the map:
p(xt;m j z1:t; u1:t)
This particular code solves the online SLAM problem.
Here xt is the pose at time t, m is the map, and z1:t and u1:t are the measurements and controls, respectively.
This problem is called the online SLAM problem since it only involves the estimation of variables that persist at time t.
Many algorithms for the online SLAM problem are incremental, they discard past measurements and controls once they have been processed.</p>

## General assumptions:
### Feature-based maps:
<p align="justify">

Maps, in the EKF, are composed of point landmarks.
For computational reasons, the number of point landmarks is usually small (e.g., smaller than 1,000).
Further, the EKF approach tends to work well the less ambiguous the landmarks are(in the code the landmarks are given, hence ideally the error associated is 0).
For this reason, EKF SLAM requires significant engineering of feature detectors, sometimes using artificial beacons or landmarks as features.</p>

### Gaussian noise:
<p align="justify">

As any EKF algorithm, EKF SLAM makes a Gaussian noise assumption for the robot motion and the perception.
The amount of uncertainty in the posterior must be relatively small, since otherwise the linearization in EKFs tend to introduce intolerable errors.</p>

## Simulation colour code:
<p align="center">

![image](https://user-images.githubusercontent.com/77194049/128481462-208536b9-9b33-4853-a389-2b5067b3c65a.png)</p>


<p align="justify">

* Black stars: landmarks{given landmarks}
* Black line: dead reckoning{prediction}
* Cyan line: ground truth{true position}
* Red line: EKF SLAM position estimation{esti,mation}
* Magenta crosses: estimates of landmark positions </p>

## Variables Used
```
show_animation = True
State_Covariance= np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2
Time_ticks = 0.1  # time tick [s] /// delta_t used
Simulation_Time = 50.0  
Sensor_Range = 20.0  
Mahalanobis_threshold_distance = 1.0 
n_state_matrix = 3 
n_Landmarks_matrix = 2 
```
## Input:
```
def calc_input():
    v = 1.0  # [m/s] ---linear velocity
    angular_velocity = 0.1  # [rad/s]---- angular velocity
    u = np.array([[v, angular_velocity]]).T
    return u
```

## Mathematical model:

### Motion model:
<p align="justify">

Since the bot used is TURTLEBOT-3 which is a differential drive bot, the motion model is framed as a differential drive model.</p>
```
def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[Time_ticks * math.cos(x[2, 0]), 0],
                  [Time_ticks * math.sin(x[2, 0]), 0],
                  [0.0, Time_ticks]])
    #upscaled matrix
    x = (F @ x) + (B @ u)
    return x
```

#### Jacobian of the motion model matrix:
<p align="center">

![image](https://user-images.githubusercontent.com/77194049/128479963-cd1866b6-0803-468f-a6c8-b2e0e5b29a46.png)</p>

<p align="justify">

The robot R moves according to a control signal u and a perturbation n and updates its state,
R ← f(R, u, n) 
The control signal is often the data from the sensors. 
It can also be the control data sent by the computer to the robot’s wheels.
And it can also be void, in case the motion model does not take any control input.</p>

```
def jacob_motion(x, u):
    #upscale matrix__Fx=(I[3x3],0[3x2n])
    Fx = np.hstack((np.eye(n_state_matrix), np.zeros(
        (n_state_matrix, n_Landmarks_matrix * calc_n_land_marks(x)))))
    #jF---jacobian of F(low dimwnsion//not up scaled )
    jF = np.array([[0.0, 0.0, -Time_ticks *u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, Time_ticks * u[0, 0]* math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)
    #I(3x3)+(Fx).T*jF.Fx---- without error model incorporated
    A = np.eye(n_state_matrix) + Fx.T @ jF @ Fx
    #A=jacob_motion_model(low dimension)
    return A, Fx
```

### Observation model
<p align="center">

The robot R observes a landmark Li that was already mapped by means of one of its sensors S.
It obtains a measurement yi, yi = h(R, S,Li)</p>

```
def observation(True_x, xd, u, map_of_landmarks):
    #updated pose value (without any error introduced)
    True_x = motion_model(True_x, u)

    # add noise to gps x-y///z={range,bearing,index of the observed landmark}
    z = np.zeros((0, 3))

    for i in range(len(map_of_landmarks[:, 0])):
        #delta_x
        dx = map_of_landmarks[i, 0] - True_x[0, 0]
        #delta_y
        dy = map_of_landmarks[i, 1] - True_x[1, 0]
        #eucledian distance
        d = math.hypot(dx, dy)
        theta = angle_normalise(math.atan2(dy, dx) - True_x[2, 0])
        
        if d <= Sensor_Range:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise---gausian noise
            theta_n = theta + np.random.randn() * Q_sim[1, 1] ** 0.5  # add noise------gausian noise
            zi = np.array([dn, theta_n, i])
            z = np.vstack((z, zi))

    # add noise to input------gausian noise
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5,
        u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5]]).T
    #xd--- dead reckoning (plot only from odometry information)
    xd = motion_model(xd, ud)
    return True_x, z, xd, ud
```

#### Jacobian of the observation model
```
def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    A = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])
    A = A / q
    #nLM ---- number of landmarks
    nLM = calc_n_land_marks(x)
    #horizontal stacking of the matrices to get the factor matrix
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
    F = np.vstack((F1, F2))
    #H is the upscaled jacobian matrix
    H = A @ F
    return H
```
## Innovation matrix
<p align="center"> 

Using the estimate of the current position it is possible to estimate where the landmark should be.
There is usually some difference, this is called the innovation.</p>

```
def calc_innovation(land_marks, X_estimation, X_uncertainty, z, LMid):
    delta = land_marks - X_estimation[0:2]
    q = (delta.T @ delta)[0, 0]
    
    z_theta = math.atan2(delta[1, 0], delta[0, 0]) - X_estimation[2, 0]
    #updated range and bearing from the early info
    zp = np.array([[math.sqrt(q), angle_normalise(z_theta)]])
    #the difference in observed and predicted:y=[delta_x,delta_y].T
    y = (z - zp).T
    y[1] = angle_normalise(y[1])
    H = jacob_h(q, delta, X_estimation, LMid + 1)
    S = H @ X_uncertainty @ H.T + State_Covariance[0:2, 0:2]
    #s---innovation covariance
    return y, S, H
```
## Landmarks:
### Number of landmarks provided:
```
def calc_n_land_marks(x):
    #N=2n+3----N:dimension of the state vector
    n = int((len(x) - n_state_matrix) / n_Landmarks_matrix)
    return n
```
### Landmark position with respect to world map:
```
def calc_landmark_position(x, z):
    zp = np.zeros((2, 1))
    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])#x_landmark=x_bot+r*cos(bearing)
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])#y_landmark=y_bot+r*sin(bearing)
    return zp
```
### Landmark pose from state matrix:
```
def get_landmark_position_from_state(x, ind):
    land_marks = x[n_state_matrix + n_Landmarks_matrix * ind: n_state_matrix + n_Landmarks_matrix * (ind + 1), :]
    return land_marks
```
### Search for correspondences of landmark(data association):
```
def search_correspond_landmark_id(Extended_state_matrix, Extended_covariance_matrix, zi):
    nLM = calc_n_land_marks(Extended_state_matrix)#number of landmarks

    min_dist = []

    for i in range(nLM):
        land_marks = get_landmark_position_from_state(Extended_state_matrix, i)
        y, S, H = calc_innovation(land_marks, Extended_state_matrix, Extended_covariance_matrix, zi, i)
        #Mahalanobis distance=y.T @ np.linalg.inv(S) @ y
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(Mahalanobis_threshold_distance)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id
```
## Normalisation:
<p align="center"> 

Angle normalisation to make sure the angle is in the interval [-pi,pi]</p>

```
def angle_normalise(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi
```
## EXTENDED KALMAN FILTER:
<p align="center">

In EKF-SLAM, the map is a large vector stacking sensors and landmarks states, and
it is modeled by a Gaussian variable. This map, usually called the stochastic map, is
maintained by the EKF through the processes of prediction (the sensors move) and correction (the sensors observe the landmarks in the environment that had been previously
mapped).
In order to achieve true exploration, the EKF machinery is enriched with an extra
step of landmark initialization, where newly discovered landmarks are added to the map.
Landmark initialization is performed by inverting the observation function and using it
and its Jacobians to compute, from the sensor pose and the measurements, the observed
landmark state and its necessary co- and cross-variances with the rest of the map. These
relations are then appended to the state vector and the covariances matrix</p>

```
def ekf_slam(X_estimation, X_uncertainty, u, z):
    #////z current or new measurement read by the sensors//////
    # Prediction or pose estimation
    S = n_state_matrix
    #Fx matrix to upscale the 3d matrix to a (2n+3)d matrix
    #jacob_motion_model_matrix(state matrix, controll data)
    A, Fx = jacob_motion(X_estimation[0:S], u)
    X_estimation[0:S] = motion_model(X_estimation[0:S], u)
    #sigma = A.T*sigma*A + Noise(upscaled)
    X_uncertainty[0:S, 0:S] = A.T @ X_uncertainty[0:S, 0:S] @ A + Fx.T @ State_Covariance @ Fx
    covariance_matrix_initialised = np.eye(2)#2x2 identity matrix

    # Update
    for iz in range(len(z[:, 0])):  # for each observation
        min_id = search_correspond_landmark_id(X_estimation, X_uncertainty, z[iz, 0:2])

        nLM = calc_n_land_marks(X_estimation)
        if min_id == nLM:
            print("New Land mark deducted")
            # Extend state and covariance matrix (updating the new landmarks to the final state matrix we wish to get)
            Extended_state_matrix = np.vstack((X_estimation, calc_landmark_position(X_estimation, z[iz, :])))
            Extended_covariance_matrix = np.vstack((np.hstack((X_uncertainty, np.zeros((len(X_estimation), n_Landmarks_matrix)))),
                              np.hstack((np.zeros((n_Landmarks_matrix, len(X_estimation))), covariance_matrix_initialised))))
            X_estimation = Extended_state_matrix
            X_uncertainty = Extended_covariance_matrix
        
        land_marks = get_landmark_position_from_state(X_estimation, min_id)
        y, S, H = calc_innovation(land_marks, X_estimation, X_uncertainty, z[iz, 0:2], min_id)
        #Kit = Σ¯t HiTt (Hi*Σt*Hi.T + Qt)−1
        #S=Hi*Σt*Hi.T + Qt
        #K==kalman gain matrix
        K = (X_uncertainty @ H.T) @ np.linalg.inv(S)
        
        X_estimation = X_estimation + (K @ y)
        X_uncertainty = (np.eye(len(X_estimation)) - (K @ H)) @ X_uncertainty

    X_estimation[2] = angle_normalise(X_estimation[2])

    return X_estimation, X_uncertainty
```
## MAIN FUNCTION:
```
def main():
    print("EKF initialised....")

    time = 0.0
    """[10.0, -2.0],[15.0, 10.0],[2.0,5.0],[4.0,4.0],[-3.0,-1.0],[3.0,2.0],[3.0,-1.0],
                     [3.0, 15.0],[1.0,-3.0],[-1.0,-3.0],[-2.0,5.0],[-1.0,4.0],[2,9],[10.0, -2.0],[15.0, 10.0],
                     [3.0, 15.0],[-5.0, 20.0],[-5.0, 2.0],[6,9],[5,7],[11,5],[1.0,4.0],"""
    # map_of_landmarks positions [x, y]
    map_of_landmarks = np.array([[5,5],[5,10],[5,15],[10,5],[10,10],[10,15],[15,5],[15,10],[15,15],])

    # State Vector [x y yaw v] initialisation
    X_estimation = np.zeros((n_state_matrix, 1))
    True_x = np.zeros((n_state_matrix, 1))
    X_uncertainty = np.eye(n_state_matrix)
    
    # Dead reckoning
    X_dead_reckoning = np.zeros((n_state_matrix, 1))  

    # history(h)////past or info from (0,t-1) time steps
    #information update
    hX_estimation = X_estimation
    hTrue_x = True_x
    hX_dead_reckoning = True_x

    while Simulation_Time >= time:
        time += Time_ticks
        u = calc_input()
        
        True_x, z, X_dead_reckoning, ud = observation(True_x, X_dead_reckoning, u, map_of_landmarks)
        X_estimation, X_uncertainty = ekf_slam(X_estimation, X_uncertainty, ud, z)

        x_state = X_estimation[0:n_state_matrix]

        # store data history which will be updated for next iteration
        hX_estimation = np.hstack((hX_estimation, x_state))
        hX_dead_reckoning = np.hstack((hX_dead_reckoning, X_dead_reckoning))
        hTrue_x = np.hstack((hTrue_x, True_x))
        
        #final plotting
        if show_animation:  # pragma: no cover
            
            plt.cla()
            plt.plot(map_of_landmarks[:, 0], map_of_landmarks[:, 1], "*k")
            plt.plot(X_estimation[0], X_estimation[1], ".r")

            # plot landmark
            for i in range(calc_n_land_marks(X_estimation)):
                plt.plot(X_estimation[n_state_matrix + i * 2],
                         X_estimation[n_state_matrix + i * 2 + 1], "xm")
                
            #trajectories plot
            plt.plot(hTrue_x[0, :],
                     hTrue_x[1, :], color="c")
            plt.plot(hX_dead_reckoning[0, :],
                     hX_dead_reckoning[1, :], "-k")
            plt.plot(hX_estimation[0, :],
                     hX_estimation[1, :], "-r")
            
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
    ```


