% Checks if two lines intersect
function intersect = checkIntersect(pa, pb)
    [qq, intersection] = lineIntersect3D(pa, pb);
    intersect = false;
        if isequal(intersection, [0;0;0]) 
            disp("intersecting!");
            disp([pa;pb]);
            disp(qq);
            intersect = true;
        end
end