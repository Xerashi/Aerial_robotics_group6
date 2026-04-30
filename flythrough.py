    # brief descent after passing gate
    self.post_fly_descent = time.time()

    #put into __init__
    self.post_fly_descent = None

    if self.post_fly_descent is not None:
        if time.time() - self.post_fly_descent < 1.0:  # 1 second drop
            cmd.linear.z = -0.15   # gentle descent
            self.cmd_vel_pub.publish(cmd)
            self.display_ui(annotated_frame)
            return
        else:
            self.post_fly_descent = None