function [ delta ] = LMupdate(lambda, J, r)
  tJ = transpose(J);
  delta = -inv(tJ*J - lambda * eye(size(J))) * tJ * r;
end


