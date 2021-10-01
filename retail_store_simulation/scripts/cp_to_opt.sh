sudo cp /home/user/ws/src/stocking-challenge/retail_store_simulation/worlds/stocking_challenge.world /opt/pal/ferrum/share/pal_gazebo_worlds/worlds/stocking_challenge.world 
sudo cp -r /home/user/ws/src/stocking-challenge/retail_store_simulation/models/* /opt/pal/ferrum/share/pal_gazebo_worlds/models/.
mkdir -p /home/user/.pal/tiago_dual_maps/configurations
cp -r /home/user/ws/src/stocking-challenge/retail_store_simulation/maps/map /home/user/.pal/tiago_dual_maps/configurations/stocking_challenge
cd /home/user/.pal && ln -s tiago_dual_maps maps
echo 'source /home/user/sim_ws/devel/setup.bash' >> /home/user/.bashrc