function isequal = matrix_iseq(A, B)
isequal = true;
for row = 1:size(A,1)
    for col = 1:size(A,2)
        if abs(A(row,col)-B(row,col)) > 0.01
            isequal = false;
        end
    end
end
end