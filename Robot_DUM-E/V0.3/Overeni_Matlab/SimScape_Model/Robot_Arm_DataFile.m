% Simscape(TM) Multibody(TM) version: 7.0

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(16).translation = [0.0 0.0 0.0];
smiData.RigidTransform(16).angle = 0.0;
smiData.RigidTransform(16).axis = [0.0 0.0 0.0];
smiData.RigidTransform(16).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [2.2204460492503131e-15 0 0];  % mm
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = 'B[Arm1:1:-:Base2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-6.7922289602854926e-14 70.699999999999989 -2.9843179349835048e-15];  % mm
smiData.RigidTransform(2).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(2).axis = [1 8.6100729901139134e-34 1.4417335478223438e-17];
smiData.RigidTransform(2).ID = 'F[Arm1:1:-:Base2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-4.4408920985006262e-15 4.9999999999999512 0];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931962;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962595 -0.5773502691896254 0.57735026918962595];
smiData.RigidTransform(3).ID = 'B[Base2:1:-:Base:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-4.0856207306205761e-14 54.999999999999929 6.3108872417680944e-30];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931962;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962595 -0.57735026918962551 0.57735026918962595];
smiData.RigidTransform(4).ID = 'F[Base2:1:-:Base:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [190.00000000000006 -117.00000000000001 8.8817841970012523e-15];  % mm
smiData.RigidTransform(5).angle = 9.0943571039422616e-16;  % rad
smiData.RigidTransform(5).axis = [0.87266860228675058 0.48831292280963534 1.9377135502524039e-16];
smiData.RigidTransform(5).ID = 'B[Arm2:1:-:Arm1:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [265.00000000000011 -1.2612133559741778e-13 -9.9475983006414026e-14];  % mm
smiData.RigidTransform(6).angle = 3.1415926535897856;  % rad
smiData.RigidTransform(6).axis = [1 5.3223936829488126e-30 1.4119858032396972e-15];
smiData.RigidTransform(6).ID = 'F[Arm2:1:-:Arm1:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [35.000000000000171 35.000000000000178 -99.499999999999957];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [-0.57735026918962562 -0.57735026918962562 0.57735026918962595];
smiData.RigidTransform(7).ID = 'B[Wirst1:1:-:Arm2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-3.1707969583294471e-13 3.3084646133829665e-13 35.00000000000059];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897927;  % rad
smiData.RigidTransform(8).axis = [1 4.0878428789637399e-31 1.1214504090150993e-15];
smiData.RigidTransform(8).ID = 'F[Wirst1:1:-:Arm2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [3.3306690738754696e-15 0 7.5];  % mm
smiData.RigidTransform(9).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(9).axis = [1 0 0];
smiData.RigidTransform(9).ID = 'B[EndEffector:1:-:Wirst2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [7.1054273576010019e-14 13.549999999999907 107.4999999999999];  % mm
smiData.RigidTransform(10).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(10).axis = [1 -8.855732803972193e-33 -2.2100322733554997e-16];
smiData.RigidTransform(10).ID = 'F[EndEffector:1:-:Wirst2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [0 0 2.2204460492503131e-15];  % mm
smiData.RigidTransform(11).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(11).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(11).ID = 'B[Wirst2:1:-:Wirst1:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-3.3306690738754696e-14 35.000000000000163 3.7525538232330291e-14];  % mm
smiData.RigidTransform(12).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(12).axis = [-0.57735026918962584 -0.57735026918962673 -0.57735026918962495];
smiData.RigidTransform(12).ID = 'F[Wirst2:1:-:Wirst1:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-12.014898631318065 -36.212592963226122 3.5527136788005009e-14];  % mm
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = 'RootGround[Base:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [-12.014898631318154 84.487407036773845 4.0099040830503645e-14];  % mm
smiData.RigidTransform(14).angle = 3.1236606947194434;  % rad
smiData.RigidTransform(14).axis = [-0.11405330948876174 -0.0010293444311837581 0.99347409782273777];
smiData.RigidTransform(14).ID = 'RootGround[Arm1:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [-81.875621582315702 200.85505465502962 -16.254600563953176];  % mm
smiData.RigidTransform(15).angle = 2.9130113739890846;  % rad
smiData.RigidTransform(15).axis = [0.014129181916445542 0.99989886244670578 0.0016220653980565671];
smiData.RigidTransform(15).ID = 'RootGround[Arm2:1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [13.47205903665156 161.58790117972421 5.9300891686135779];  % mm
smiData.RigidTransform(16).angle = 1.342780336216671;  % rad
smiData.RigidTransform(16).axis = [0.027208125629631322 0.99939630311537286 -0.021604333339590123];
smiData.RigidTransform(16).ID = 'RootGround[Wirst1:1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(7).mass = 0.0;
smiData.Solid(7).CoM = [0.0 0.0 0.0];
smiData.Solid(7).MoI = [0.0 0.0 0.0];
smiData.Solid(7).PoI = [0.0 0.0 0.0];
smiData.Solid(7).color = [0.0 0.0 0.0];
smiData.Solid(7).opacity = 0.0;
smiData.Solid(7).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.6465805835109445;  % kg
smiData.Solid(1).CoM = [7.3520704165789365e-09 26.511186812906811 1.1956290000377091e-13];  % mm
smiData.Solid(1).MoI = [809.65874871679557 1308.2360813833131 809.65867165969053];  % kg*mm^2
smiData.Solid(1).PoI = [2.0495016352443409e-12 5.0931703304967646e-12 7.1856249908657665e-09];  % kg*mm^2
smiData.Solid(1).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'Base.ipt_{97301F3A-45DA-231D-BD64-52B6F7617D27}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.47892759359157799;  % kg
smiData.Solid(2).CoM = [-4.9201796980463594e-10 47.942117934317245 9.4951169440085676e-15];  % mm
smiData.Solid(2).MoI = [640.14429124161654 413.83996669608348 775.13260307187409];  % kg*mm^2
smiData.Solid(2).PoI = [-1.457823694439992e-13 -1.8189894035682307e-13 -1.0253027845103366e-08];  % kg*mm^2
smiData.Solid(2).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'Base2.ipt_{9B5A93A1-4750-961C-011D-56BFA0FC2BF6}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 1.1467289154439271;  % kg
smiData.Solid(3).CoM = [150.34739790246641 -1.5862418563341805e-14 7.9312092816709027e-15];  % mm
smiData.Solid(3).MoI = [914.34520512420841 8861.0571197616609 9122.850679399251];  % kg*mm^2
smiData.Solid(3).PoI = [-5.4569682106375712e-13 2.8225931410130803e-12 -1.2796117135161046e-12];  % kg*mm^2
smiData.Solid(3).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'Arm1.ipt_{D6919D19-4E81-23FF-7AE2-8B8C0D3725C7}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 1.30484200520032;  % kg
smiData.Solid(4).CoM = [91.133812221275221 -33.159696937448963 6.9701519275761061e-15];  % mm
smiData.Solid(4).MoI = [3041.4015383049882 5364.514427905684 7343.9086859311392];  % kg*mm^2
smiData.Solid(4).PoI = [3.3363931203216545e-12 3.0116444779312151e-12 2469.9860426267564];  % kg*mm^2
smiData.Solid(4).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'Arm2.ipt_{24867B2D-4D0F-989D-1A82-8682DA839539}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.52164907754058099;  % kg
smiData.Solid(5).CoM = [1.096225064627365e-11 34.999999999999993 -31.557463039218817];  % mm
smiData.Solid(5).MoI = [1070.1637779452876 1112.1475258087669 461.95473182309416];  % kg*mm^2
smiData.Solid(5).PoI = [-2.7284841053187846e-13 -2.0956353988284566e-10 0];  % kg*mm^2
smiData.Solid(5).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'Wirst1.ipt_{F7FAA561-407C-82F1-C68A-9B84EDE4A4A4}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.50915430307786913;  % kg
smiData.Solid(6).CoM = [-7.4977208802942971e-07 0.45177813916240772 53.040309961531491];  % mm
smiData.Solid(6).MoI = [971.29475556334967 780.09654216828142 691.56008384156826];  % kg*mm^2
smiData.Solid(6).PoI = [-10.973879431489408 1.3226287811539148e-05 1.6405234281739926e-05];  % kg*mm^2
smiData.Solid(6).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = 'Wirst2.ipt_{9507E5C0-413F-368D-3B4B-B8A312748D40}';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.04322546838553229;  % kg
smiData.Solid(7).CoM = [0.0019950471517630177 -1.8858265607698242e-09 13.202090126360957];  % mm
smiData.Solid(7).MoI = [12.381864673843493 12.382326622691455 11.158563542998404];  % kg*mm^2
smiData.Solid(7).PoI = [-1.3317135908475094e-10 -0.0057600515983074977 1.0825205128099115e-11];  % kg*mm^2
smiData.Solid(7).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = 'EndEffector.ipt_{D3F9522C-487E-CA54-6E2F-8DA5F43DAFF8}';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(6).Rz.Pos = 0.0;
smiData.RevoluteJoint(6).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 1.0341741179275681;  % deg
smiData.RevoluteJoint(1).ID = '[Arm1:1:-:Base2:1]';

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.RevoluteJoint(2).Rz.Pos = 166.90194420418086;  % deg
smiData.RevoluteJoint(2).ID = '[Base2:1:-:Base:1]';

smiData.RevoluteJoint(3).Rz.Pos = -0.58496687135975411;  % deg
smiData.RevoluteJoint(3).ID = '[Arm2:1:-:Arm1:1]';

smiData.RevoluteJoint(4).Rz.Pos = 90.857642947825724;  % deg
smiData.RevoluteJoint(4).ID = '[Wirst1:1:-:Arm2:1]';

smiData.RevoluteJoint(5).Rz.Pos = -176.77053104384146;  % deg
smiData.RevoluteJoint(5).ID = '[EndEffector:1:-:Wirst2:1]';

smiData.RevoluteJoint(6).Rz.Pos = -2.2155848319402618;  % deg
smiData.RevoluteJoint(6).ID = '[Wirst2:1:-:Wirst1:1]';

