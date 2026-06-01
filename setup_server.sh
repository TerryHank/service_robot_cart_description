cat: 'C:/Users/Virtual/ros2_ws/src/service_robot_cart_description/setup_server.sh': No such file or directory
echo ""
echo "=== Authorizing Hermes SSH key ==="
mkdir -p ~/.ssh
chmod 700 ~/.ssh
echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIEWjWqXrseL7LxJgEJcg/XBXs/2DzJYNThKtfc+WOhm9 virtual@LabPC" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
echo "SSH key added"
