function starterCodeForBridgeOfDoomQEA2020()
% Insert any setup code you want to run here

% define u explicitly to avoid error when using sub functions
% see: https://www.mathworks.com/matlabcentral/answers/268580-error-attempt-to-add-variable-to-a-static-workspace-when-it-is-not-in-workspace
b = [];
t = [];
% u will be our parameter

%hit spacebar before and after for data collection
%save bridgeData.mat
%collectDataset_sim('bridgeData.mat')

syms b t;

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*((b*t)+1.4)) -0.99*sin((b*t)+1.4) 0];

% tangent vector
dr = diff(R,t);

% normalized tangent vector
T_hat = dr/norm(dr);
dT_dt = simplify(diff(T_hat, t));
pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);
% calculate velocities to travel along curve
beta_num = 0.2;
d = 0.235;
w = simplify(cross(T_hat, dT_dt));
LS = norm(dr);
VR = simplify(LS - (.5 .* (w(:,3) * d)));
VL = simplify(LS + (.5 .* (w(:,3) * d)));
t_num = linspace(0, 3.2 / beta_num, 310);
x_num = linspace(0, 3.2 / beta_num, 10);

%figure
for n=1:length(t_num)
    r_num(n,:) = double(subs(R, {b, t}, {beta_num, t_num(n)}));
end
%plot(r_num(:,1), r_num(:,2))

VL_f = subs(VL, b, beta_num);
VR_f = subs(VR, b, beta_num);

%figure
%fplot(VR_f,'r-')
%hold on, axis([0,3.2/beta_num, 0, 2.5]);
%fplot(VL_f,'g')
%xlabel('time (s)')
%ylabel('velocity (m/s)')
%hold off

bridgeStart = double(subs(R,{b, t},{beta_num, 0}));
startingThat = double(subs(T_hat,{b, t},{beta_num, 0}));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);
start = rostime('now');
% time to drive!!
for n=1:length(t_num)
    goMsg = rosmessage(pub);
    current = rostime('now');
    elapsed = current - start;
    VR_go = double(subs(VR_f, {t}, {elapsed.seconds}));
    VL_go = double(subs(VL_f, {t}, {elapsed.seconds}));
    goMsg.Data = [VR_go VL_go];
    send(pub, goMsg);
    pause((3.2 / beta_num) / length(t_num))
end
% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
end
