function rot_mat = AngleToRot(angle)
    rot_mat = [cos(angle), -sin(angle);
               sin(angle),  cos(angle)];
end


