% Edit this function to implement alternative control schemes
%  *** Add code in the switch statement starting at line 19 ***
function robot=drive_robot(robot,T,dT,ctrl_enable)

% Initializing
DC_ideal=getDC(robot.setSpeed,robot.r,robot.wMax); % Get duty cycle to initialize motors
robot.rMot = DC_ideal;
robot.lMot = DC_ideal;

% Working out timings
Tstep=T(2); % Step size of simulation
ctrl_interval=round(dT/Tstep);

% Control parameters *** Add parameters here as necessary ***
Kp = 15; % Proportional control parameter
Ki = 25; % Integral control parameter
Kd = 0.1; % Derivative control parameter

propTerm = 0; % Initialize P, I, and D terms
intTerm = 0;
derTerm = 0;

figure;

%Setup for Derivative Control
spd = 0.2;
robot.setSpeed = spd;
robot.wMax = 1.5;

for i=2:length(T)
    
    % Compute change in position
    robot=newPos(robot,Tstep,i);
    
    if mod(i,ctrl_interval)< eps %If it is time to provide feedback
       switch ctrl_enable % Open loop control; no feedback
           case 0; % Open loop, do nothing
           case 1; % Proportional control
               
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               % Modify motor duty cycle accordingly
               robot.rMot = robot.rMot+Kp*enc_error;
               robot.lMot = robot.lMot-Kp*enc_error;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 2; % Proportional Integral (PI) Control
      
               enc_error = getError(robot,i,ctrl_interval); % Error signal       
               propTerm = Kp*enc_error; % Proportional control term
               
               intTerm = intTerm + (Ki*enc_error); % Integral control term
               
               robot.rMot = robot.rMot + propTerm + intTerm; % Update left and right motors with P and I terms
               robot.lMot = robot.lMot - propTerm - intTerm;
               
               robot = checkDC(robot); % Checking if PWM is within bounds
               
           case 3;  %Proportional Derivative (PD) Control
               enc_error = getError(robot,i,ctrl_interval); % Error signal
               propTerm = Kp*enc_error; % Proportional control term
               
               vL=(robot.wMax*robot.lMot*1.02*2*pi*robot.r); % Tangential speed of left and right wheels (m/s) 
               vR=(robot.wMax*robot.rMot*2*pi*robot.r);
               averagevel = (vL+vR)/2; % Average speed of left and right wheels
               dInput = robot.setSpeed - averagevel; % Derivative term
               derTerm = Kd*dInput; % Multiply by gain
               
               robot.rMot = robot.rMot + propTerm - derTerm; % Update left and right motors with P and D terms
               robot.lMot = robot.lMot - propTerm + derTerm;
           
               robot = checkDC(robot); % Checking if PWM is within bounds
               
           case 4;  %Proportional Integral Derivative (PID) Control
               enc_error = getError(robot,i,ctrl_interval); % Error signal
               propTerm = Kp*enc_error; % Proportional control term
               
               intTerm = intTerm + (Ki*enc_error); % Integral control term
               
               vL=(robot.wMax*robot.lMot*1.02*2*pi*robot.r); % Tangential speed of left and right wheels (m/s) 
               vR=(robot.wMax*robot.rMot*2*pi*robot.r);
               averagevel = (vL+vR)/2;
               dInput = robot.setSpeed - averagevel; 
               derTerm = Kd*dInput; % Derivative Control Term
               
               robot.rMot = robot.rMot + propTerm + intTerm - derTerm; % Update left and right motors with P,I, and D terms
               robot.lMot = robot.lMot - propTerm - intTerm + derTerm;
               
               robot = checkDC(robot); % Checking if PWM is within bounds
               
           case 5; % Integral Control
               
               enc_error = getError(robot,i,ctrl_interval); % Error signal       
               
               intTerm = intTerm + (Ki*enc_error); % Integral control term
               
               robot.rMot = robot.rMot + intTerm; % Update left and right motors with P and I terms
               robot.lMot = robot.lMot - intTerm;
               
               robot = checkDC(robot); % Checking if PWM is within bounds
               
               % The assignment does say at least 2 control schemes, but
               % only integral control was tested and yielded high
               % frequency oscillations but a generally low error path
           
       end
       
           % Code to visualize robot as it steps
%     clf; hold on;
%     scatter(robot.path(1,i),robot.path(2,i),'b');
%     scatter(robot.lWheel(1,i),robot.lWheel(2,i),'k');
%     scatter(robot.rWheel(1,i),robot.rWheel(2,i),'k');
%     axis tight
%     plot(xlim, [0 0], '-r')
%     xlim([-0.1 1]); ylim([-.2 .2]);
       
    end

end
end
