close all; 
figure
plot(misure(:,1))
hold on;
plot(misure(:,2))
legend('not filtered', 'filtered')
figure
plot(misure(:,3))