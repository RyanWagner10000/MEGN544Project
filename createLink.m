% createLink will create a struct with the associated data
% 
% L = createLink(a, d, alpha, theta, offset, centOfMass, mass, intertia)
% This function takes the input arguements and sets up an object with the
% associated values. This will make it easier to model and calculate
% multi-link assemblies.
% 
% L             = struct of data for a particular link
% 
% a             = DH parameter a (meters)
% d             = DH parameter d (meters)
% alpha         = DH parameter alpha (radians)
% theta         = joint angle provided by encoder (radians)
% offset        = the numebr of radians between encoder orientation and DH
% centOfMass    = position of links com
% mass          = link mass (kg)
% inertia       = link mass moment of inertia (kg m^2)
% isRotary      = 1 if rotational, 0 if prismatic, -1 if static
% 
% Ryan Wagner
% 10821236
% MEGN 544
% October 15, 2021

function L = createLink(a, d, alpha, theta, offset, centOfMass, mass, intertia)
L.a = a;
L.d = d;
L.alpha = alpha;
L.theta = theta;
L.offset = offset;
L.com = centOfMass;
L.mass = mass;
L.inertia = intertia;
if (isempty(theta))
    L.isRotary = 1;
elseif (isempty(d))
    L.isRotary = 0;
else
    L.isRotary = -1;
end
end