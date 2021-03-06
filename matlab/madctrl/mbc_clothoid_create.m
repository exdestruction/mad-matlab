function track = mbc_clothoid_create(track, a, rad, width, opening)
% track = mbc_clothoid_create(track, a, rad, width, opening) adds a clothoid arc 
% as a new segment to track
% 
%   track - existing track created by mbc_track_create.
%           The return value track contains the original track plus
%           the new segment.
%   a - clothoid parameter
%   rad - arc length [ rad ]
%   width - track width [ m ]
%
%   If rad > 0 then mbc_circle_create creates a left turn.
%   If rad < 0 then mbc_circle_create creates a right turn.

    
    cnt = mbc_track_get_cnt(track);
    p = track.points{cnt+1};

    % path length
    xe = sqrt(abs(2*rad/a)); 

    if opening ~= 1
        % functions for closing clothoids
        fun_cos = @(eps) cos((sign(rad)*a/2)*eps.^2 + p.psi);        
        fun_sin = @(eps) sin((sign(rad)*a/2)*eps.^2 + p.psi);
    else
        %functions for opening clothoids
        fun_cos = @(eps) cos((-a * sign(rad)/2)*eps.^2 + sign(rad) * a * eps * xe + p.psi);       
        fun_sin = @(eps) sin((-a * sign(rad)/2)*eps.^2 + sign(rad) * a * eps * xe + p.psi);
    end
    
    % calculating final s1, s2 coordinates
    s1 = p.s1 + integral(fun_cos, 0, xe);
    s2 = p.s2 + integral(fun_sin, 0, xe);
    
    
    % append points structure with the final coordinates
    track.points{cnt+2} = struct( ...
        's1', s1, ...
        's2', s2, ...
        'psi', p.psi + rad, ...
        'x', p.x + xe);
    track.tracks{cnt+1} = struct(...
        'type', 'clothoid', ...
        'a', sign(rad) * a, ...
        'xe', xe, ...
        'w', width, ...
        'opening', opening);
end