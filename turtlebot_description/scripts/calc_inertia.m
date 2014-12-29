% Calculate moments of inertia for 3D sensors and TB2 poles and stacks.
% Formulas extracted from: http://en.wikipedia.org/wiki/List_of_moments_of_inertia
% TODO: do also for turtlebot 1 poles and stacks.

% Poles are approximated as solid cylinder of radius r, height h and mass m

% Base poles
m = 0.008;
r = 0.006;
h = 0.0492;

Ix = (m*(3*r^2 + h^2))/12
Iy = Ix
Iz = (m*r^2)/2

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)

% Middle poles
m = 0.012;
r = 0.006;
h = 0.0608;

Ix = (m*(3*r^2 + h^2))/12
Iy = Ix
Iz = (m*r^2)/2

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)

% Long poles
m = 0.060;
r = 0.006;
h = 0.2032;

Ix = (m*(3*r^2 + h^2))/12
Iy = Ix
Iz = (m*r^2)/2

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)

% Kinect poles
m = 0.020;
r = 0.006;
h = 0.0936;  % not really... are much shorter!

Ix = (m*(3*r^2 + h^2))/12
Iy = Ix
Iz = (m*r^2)/2

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)


% Plates, approximated as a thin, solid disk of radius r and mass m:
m = 0.520;
r = 0.160;

Ix = (m*r^2)/4
Iy = Ix
Iz = (m*r^2)/2

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)

% Cameras, approximated as a solid cuboid of height h, width w, and depth d, and mass m:

% Kinect
m = 0.564;
h = 0.073;
w = 0.27794;
d = 0.07271;

Ix = (m*(w^2 + h^2))/12
Iy = (m*(h^2 + d^2))/12
Iz = (m*(w^2 + d^2))/12

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)

% Asus
m = 0.170;
h = 0.072;
w = 0.276;
d = 0.073;

Ix = (m*(w^2 + h^2))/12
Iy = (m*(h^2 + d^2))/12
Iz = (m*(w^2 + d^2))/12

fprintf('%.9f\n%.9f\n%.9f\n', Ix, Iy, Iz)