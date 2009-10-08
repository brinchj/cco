function [ info ] = collision_detection( config )
% COLLISION_DETECTION - Perform Collision Detection.
%   [ info ] = create_mesh( config )
% INPUT:
%       config - A simulation configuration object.
% RESULTS:
%       info   - A collision detection object with proximity onfo about all objects.
% Copyright 2009, Kenny Erleben, DIKU.

T = DelaunayTri(config.X,config.Y);
O = edges(T);

% We need to filter out all entries between fixed objects!
Ew =  config.W( O(:,1))+config.W( O(:,2));
O = O( Ew>0,: );

% Temporaries for more readable  code
Xv = config.X;
Yv = config.Y;
R  = config.R;

% compute edge vectors
dx = Xv( O(:,1) ) - Xv( O(:,2) );
dy = Yv( O(:,1) ) - Yv( O(:,2) );

% Compute edge lengths
Le = sqrt( dx.^2 + dy.^2 );

% Compute separation distance
D = Le - R( O(:,1 ) ) -  R( O(:,2 ) ); 

% Compute contact normals
Nx =  dx ./Le;
Ny =  dy ./Le;

% Compute contact points
R1 = R(O(:,1));
R2 = R(O(:,2));
w1 = R2 ./ (R1+R2);
w2 = R1 ./ (R1+R2);
X = w1.*Xv(O(:,1)) + w2.*Xv(O(:,2)); 
Y = w1.*Yv(O(:,1)) + w2.*Yv(O(:,2)); 

info = struct( 'X', X, 'Y', Y, 'D', D, 'Nx', Nx, 'Ny', Ny, 'O', O );
