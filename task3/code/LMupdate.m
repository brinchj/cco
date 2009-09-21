function [ delta ] = LMupdate(lambda, J, r)
  tJ = transpose(J);
  delta = -inv(tJ*J - lambda * eye(length(J))) * tJ * r;
end


