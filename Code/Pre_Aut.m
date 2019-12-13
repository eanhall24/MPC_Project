function PreS=Pre_Aut(A,S)
% works with polytope which are also not full dimensional
nx=size(A,2);
PreS=Polyhedron('H',[S.H(:,1:nx)*A S.H(:,nx+1)],...
'He',[S.He(:,1:nx)*A S.He(:,nx+1)]);
end