function points = mbc_clothoid_get_points(track, idx, xstart, dx, alpha)
% points = mbc_clothoid_get_points(track, idx, xstart, dx, alpha)
% computes the accurate line points on the track segment.
%
%   track - track object
%   idx - the index of the track segment
%   xstart - the arc length of the segment start positon [ m ]
%   dx - the arc distance of two neighbour points [ m ]
%   alpha - the line position on the road [ 0 ; 1 ]
%   points - the points as a 3 x n matrix
%
%   points(1, :) is the arc length at each point
%   points(2, :) is the x position
%   points(3, :) is the y position
%
%   If alpha == 0 then the right line points are generated.
%   If alpha == 0.5 then the center line points are generated.
%   If alpha == 1 then the left line points are generated.

    if nargin < 4
        alpha = 0.5; % return center line points
    end

    p = track.points{idx};
    t = track.tracks{idx};

    idx = 0:floor((abs(t.xe) - (xstart - p.x) + mbc_cmp_eps)/dx);
    x = (xstart - p.x) + dx * idx;
	
	if t.opening ~= 1
		psi = p.psi + (t.a * x.^2) / 2;
		% functions for closing clothoids
        fun_cos = @(eps) cos((t.a/2)*eps.^2 + p.psi);        
        fun_sin = @(eps) sin((t.a/2)*eps.^2 + p.psi);     
	else 
		%functions for opening clothoids
		psi = p.psi + ((t.a * t.xe.^2) / 2) - (t.a/2) * (t.xe - x).^2;
        fun_cos = @(eps) cos((-t.a/2)*eps.^2 + t.a*eps*t.xe + p.psi);       
        fun_sin = @(eps) sin((-t.a/2)*eps.^2 + t.a*eps*t.xe + p.psi);
	end
	
	%computing point of clothoid
	for i = 1:length(x)
		s1(i) = p.s1 + integral(fun_cos, 0, x(i)) + cos(pi/2 + psi(i)) * (alpha - 0.5) * t.w;
        s2(i) = p.s2 + integral(fun_sin, 0, x(i)) + sin(pi/2 + psi(i)) * (alpha - 0.5) * t.w;
	end
		
	%appending data structure points
    points = [p.x + x; s1; s2]; 
end