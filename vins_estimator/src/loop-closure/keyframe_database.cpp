#include "keyframe_database.h"
KeyFrameDatabase::KeyFrameDatabase()
{
	posegraph_visualization = new CameraPoseVisualization(0.0, 0.0, 1.0, 1.0);
	posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
}
void KeyFrameDatabase::add(KeyFrame* pKF)
{
	ROS_DEBUG("add keyframe begin!");
	unique_lock<mutex> lock(mMutexkeyFrameList);
	keyFrameList.push_back(pKF);
	lock.unlock();
	Vector3d P;
	Matrix3d R;
	pKF->getPose(P, R);
	Quaterniond Q;
	Q = R;

	total_length += (P - last_P).norm();
	last_P = P;
	//posegraph_visualization->add_pose(P, Q);
/*
	//draw local connection
	list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	list<KeyFrame*>::reverse_iterator lrit;
	for (; rit != keyFrameList.rend(); rit++)  
    {  
        if ((*rit) == pKF)
        {
        	lrit = rit;
        	lrit++;
        	for (int i = 0; i < 4; i++)
        	{
        		if (lrit == keyFrameList.rend())
        			break;
        		Vector3d conncected_P;
        		Matrix3d connected_R;
        		(*lrit)->getPose(conncected_P, connected_R);
        		posegraph_visualization->add_edge(P, conncected_P);
        		lrit++;
        	}
        	break;
        }
    } 
*/
	// add key frame to path for visualization
	nav_msgs::Odometry odometry;
	odometry.header.stamp = ros::Time(pKF->header);
	odometry.header.frame_id = "world";
	odometry.pose.pose.position.x = P.x();
	odometry.pose.pose.position.y = P.y();
	odometry.pose.pose.position.z = P.z();
	odometry.pose.pose.orientation.x = Q.x();
	odometry.pose.pose.orientation.y = Q.y();
	odometry.pose.pose.orientation.z = Q.z();
	odometry.pose.pose.orientation.w = Q.w();

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = odometry.header;
	pose_stamped.pose = odometry.pose.pose;

	//unique_lock<mutex> mlockPath(mPath);
	//refine_path.header = odometry.header;
	//refine_path.poses.push_back(pose_stamped);
	//mlockPath.unlock();
	
	ROS_DEBUG("add keyframe end!");

}
/**
 * @brief 降低采样
 * @param erase_index 输出被删除的关键帧序号
 */
void KeyFrameDatabase::downsample(vector<int> &erase_index)
{
	ROS_DEBUG("resample keyframe begin!");

	unique_lock<mutex> lock(mMutexkeyFrameList);

	// 关键帧队列中的关键帧数量
	int frame_num = (int)keyFrameList.size();

	if (mOptimiazationPosegraph.try_lock())
	{
		erase_index.clear();
		double min_dis = total_length / (frame_num * 0.7);

		list<KeyFrame*>::iterator it = keyFrameList.begin();
		Vector3d last_P = Vector3d(0, 0, 0);

		// 遍历关键帧链表
		for (; it != keyFrameList.end(); )   
		{
			Vector3d tmp_t;
			Matrix3d tmp_r;

			// 角度在这里没有被考虑
			(*it)->getPose(tmp_t, tmp_r);

			// last_p：遍历整个链表，离it最近被保留下来的关键帧last_p
			// 计算距离
			double dis = (tmp_t - last_P).norm();

				// 当it为关键帧库中的第一帧或者是两帧之间的距离大于阀值或者是该关键帧闭环过
		    if(it == keyFrameList.begin() || dis > min_dis || (*it)->has_loop || (*it)->is_looped)
		    {
		    	last_P = tmp_t;
		    	it++;
		    }
		    else
		    {
		    	erase_index.push_back((*it)->global_index);

					// 删除关键帧
		    	delete (*it);

					// 返回在队列中被删除的下一个
		    	it = keyFrameList.erase(it);
		    }
		}
		mOptimiazationPosegraph.unlock();
	}
	else
		return;

	lock.unlock();
	ROS_DEBUG("resample keyframe end!");
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
	list<KeyFrame*>::iterator it = find(keyFrameList.begin(), keyFrameList.end(), pKF);
	assert(it != keyFrameList.end());
	if (it != keyFrameList.end())
	    keyFrameList.erase(it);
}

int KeyFrameDatabase::size()
{
	unique_lock<mutex> lock(mMutexkeyFrameList);
	return (int)keyFrameList.size();
}

void KeyFrameDatabase::getKeyframeIndexList(vector<int> &keyframe_index_list)
{
	unique_lock<mutex> lock(mMutexkeyFrameList);
	list<KeyFrame*>::iterator it = keyFrameList.begin();
	for (; it != keyFrameList.end(); it++)   
	{
		keyframe_index_list.push_back((*it)->global_index);
	}
	return;
}

KeyFrame* KeyFrameDatabase::getKeyframe(int index)
{
	unique_lock<mutex> lock(mMutexkeyFrameList);
	list<KeyFrame*>::iterator it = keyFrameList.begin();
	for (; it != keyFrameList.end(); it++)   
	{
	    if((*it)->global_index == index)
	    	break;
	}
	if (it != keyFrameList.end())
    	return *it;
    else
    	return NULL;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe()
{
	unique_lock<mutex> lock(mMutexkeyFrameList);
	list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	assert(rit != keyFrameList.rend());
    return *rit;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe(int last_index)
{
	unique_lock<mutex> lock(mMutexkeyFrameList);
	list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	for (int i = 0; i < last_index; i++)  
    {  
        rit++;
        assert(rit != keyFrameList.rend());
    } 
    return *rit;
}

/**
 * @brief 4自由度的图优化
 * @param cur_index 当前帧序列
 * @param loop_correct_t
 * @param loop_correct_r
 */
void KeyFrameDatabase::optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
	ROS_DEBUG("optimizae pose graph begin!");
	unique_lock<mutex> lock(mOptimiazationPosegraph);

	// 取得当前帧
	KeyFrame* cur_kf = getKeyframe(cur_index);

	int loop_index = cur_kf->loop_index;
	if (earliest_loop_index > loop_index || earliest_loop_index == -1)
		earliest_loop_index = loop_index;
	assert(cur_kf-> update_loop_info == 1);
	int max_length = cur_index + 1;

	// w^t_i   w^q_i
	double t_array[max_length][3];
	Quaterniond q_array[max_length];
	double euler_array[max_length][3];

	ceres::Problem problem;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

	//options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = SOLVER_TIME * 3;
	options.max_num_iterations = 5;
	ceres::Solver::Summary summary;
	ceres::LossFunction *loss_function;
	loss_function = new ceres::HuberLoss(1.0);
	//loss_function = new ceres::CauchyLoss(1.0);

	// 局部参数化，为了限制求解的范围在[-pi, pi]
	ceres::LocalParameterization* angle_local_parameterization =
	    AngleLocalParameterization::Create();

	list<KeyFrame*>::iterator it;

	int i = 0;

	// 遍历所有的关键帧
	for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
	{
		// 找到已经闭环后的关键帧
		if ((*it)->global_index < earliest_loop_index)
			continue;
		(*it)->resample_index = i;

		Quaterniond tmp_q;
		Matrix3d tmp_r;
		Vector3d tmp_t;
		(*it)->getOriginPose(tmp_t, tmp_r);
		tmp_q = tmp_r;

		t_array[i][0] = tmp_t(0);
		t_array[i][1] = tmp_t(1);
		t_array[i][2] = tmp_t(2);
		q_array[i] = tmp_q;

		// 四元数转换为欧拉角
		Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
		euler_array[i][0] = euler_angle.x();
		euler_array[i][1] = euler_angle.y();
		euler_array[i][2] = euler_angle.z();

		// 添加参数块，欧拉角和位移量
		problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
		problem.AddParameterBlock(t_array[i], 3);

		// 最早形成闭环的关键帧数据保持不变
		if ((*it)->global_index == earliest_loop_index)
		{	
			problem.SetParameterBlockConstant(euler_array[i]);
			problem.SetParameterBlockConstant(t_array[i]);
		}

		//add edge
		// 添加残差
		// 遍历该帧相邻的五个关键帧
		for (int j = 1; j < 5; j++)
		{
		  if (i - j > 0)
		  {
		    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());

		    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
		    relative_t = q_array[i-j].inverse() * relative_t;

			// 第i-j帧到i帧的yaw轴变化值
			// euler_conncected.y() euler_conncected.z()分别是第i-j帧的pitch，roll轴
			double relative_yaw = euler_array[i][0] - euler_array[i-j][0];

			// FourDOFError::Create：新建立一个FourDOFError结构体
			// relative_yaw:通过里程计得到的原始数据
			ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
		                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
			// euler_array[i-j]：原始里程计得到位姿
			// 这里没用核函数的原因是，连续帧之间进行优化是不会存在外点
		    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
		                            t_array[i-j], 
		                            euler_array[i], 
		                            t_array[i]);
		  }
		}

		//add loop edge
		if((*it)->update_loop_info)
		{
			int connected_index = getKeyframe((*it)->loop_index)->resample_index;

			assert((*it)->loop_index >= earliest_loop_index);

			Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());

			Vector3d relative_t;
			// 得到由闭环检测得到的相对旋转、位移量
			relative_t = (*it)->getLoopRelativeT();
			double relative_yaw = (*it)->getLoopRelativeYaw();

			// 比重为5，相对于FourDOFError
			ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
																	   relative_yaw, euler_conncected.y(), euler_conncected.z());
			// 使用核函数，是因为闭环检测可能存在无匹配，存在外点
			problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
														  t_array[connected_index], 
														  euler_array[i], 
														  t_array[i]);
			
		}
		if ((*it)->global_index == cur_index)
			break;
		i++;
	}

	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";

	i = 0;

	// 遍历所有关键帧，将优化的结果用于更新所有关键帧位置
	for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
	{
		if ((*it)->global_index < earliest_loop_index)
			continue;
		Quaterniond tmp_q;

		tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
		Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
		Matrix3d tmp_r = tmp_q.toRotationMatrix();
		(*it)-> updatePose(tmp_t, tmp_r);

		if ((*it)->global_index == cur_index)
			break;
		i++;
	}

	// 遍历所有关键帧，利用当前优化和优化前的Yaw轴的值，进行所有关键帧的Yaw矫正
	Vector3d cur_t, origin_t;
	Matrix3d cur_r, origin_r;
	cur_kf->getPose(cur_t, cur_r);
	cur_kf->getOriginPose(origin_t, origin_r);
	yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
	r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
	t_drift = cur_t - r_drift * origin_t;

	for (; it != keyFrameList.end(); it++)
	{
		Vector3d P;
		Matrix3d R;
		(*it)->getOriginPose(P, R);
		P = r_drift * P + t_drift;
		R = r_drift * R;
		(*it)-> updatePose(P, R);
	}
	loop_correct_t = t_drift;
	loop_correct_r = r_drift;
	ROS_DEBUG("optimizae pose graph end!");

}

void KeyFrameDatabase::updateVisualization()
{
	ROS_DEBUG("updateVisualization begin");
	unique_lock<mutex> mlockPath(mPath);
	unique_lock<mutex> mlockPosegraph(mPosegraphVisualization);
	total_length = 0;
	last_P = Vector3d(0, 0, 0);
	//update visualization
	list<KeyFrame*>::iterator it;
	posegraph_visualization->reset();
	refine_path.poses.clear();

	for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
	{

		Vector3d P;
		Matrix3d R;
		(*it)->getPose(P, R);
		Quaterniond Q;
		Q = R;

		total_length += (P - last_P).norm();
		last_P = P;
		//posegraph_visualization->add_pose(P, Q);
		/*
		list<KeyFrame*>::iterator lit;
		lit = it;
    	for (int i = 0; i < 4; i++)
    	{
    		if (lit == keyFrameList.begin())
    			break;
    		lit--;
    		Vector3d conncected_P;
    		Matrix3d connected_R;
    		(*lit)->getPose(conncected_P, connected_R);
    		posegraph_visualization->add_edge(P, conncected_P);
    	}
		*/
		// draw loop edge

		if ((*it)->update_loop_info)
		{
			
			KeyFrame* connected_KF = getKeyframe((*it)->loop_index);
			Vector3d conncected_P;
			Matrix3d connected_R;
			connected_KF->getPose(conncected_P, connected_R);
			posegraph_visualization->add_loopedge(P, conncected_P);
			
			/*
			//supposed edge
			Vector3d supposed_P;
			Vector3d relative_t;
			relative_t = (*it)->getLoopRelativeT();
			supposed_P = P - connected_R * relative_t;
			posegraph_visualization->add_edge(P, supposed_P);
			*/
			
			list<KeyFrame*>::iterator lit;
			lit = it;
			lit--;
			Vector3d P_previous;
			Matrix3d R_previous;
			(*lit)->getPose(P_previous, R_previous);
			posegraph_visualization->add_loopedge(P, P_previous);
		}

		// add key frame to path for visualization
		nav_msgs::Odometry odometry;
		odometry.header.stamp = ros::Time((*it)->header);
		odometry.header.frame_id = "world";
		odometry.pose.pose.position.x = P.x();
		odometry.pose.pose.position.y = P.y();
		odometry.pose.pose.position.z = P.z();
		odometry.pose.pose.orientation.x = Q.x();
		odometry.pose.pose.orientation.y = Q.y();
		odometry.pose.pose.orientation.z = Q.z();
		odometry.pose.pose.orientation.w = Q.w();

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header = odometry.header;
		pose_stamped.pose = odometry.pose.pose;
		refine_path.header = odometry.header;
		refine_path.poses.push_back(pose_stamped);

	}
	ROS_DEBUG("updateVisualization end");
}
/**
 * @brief 向关键帧数据库中添加闭环序号,更新闭环显示
 * @param loop_index 关键帧库中与当前关键帧匹配上的序号
 */
void KeyFrameDatabase::addLoop(int loop_index)
{
	unique_lock<mutex> lock(mPosegraphVisualization);
	if (earliest_loop_index > loop_index || earliest_loop_index == -1)
		earliest_loop_index = loop_index;

	// 当前关键帧
	KeyFrame* cur_KF = getLastKeyframe();

	// 关键帧库中与当前关键帧匹配上的关键帧
	KeyFrame* connected_KF = getKeyframe(loop_index);
	Vector3d conncected_P, P;
	Matrix3d connected_R, R;

	// 当前关键帧的变换矩阵
	cur_KF->getPose(P, R);

	// 关键帧库中关键帧的变换矩阵
	connected_KF->getPose(conncected_P, connected_R);

	// 更新显示
	posegraph_visualization->add_loopedge(P, conncected_P);
}

nav_msgs::Path KeyFrameDatabase::getPath()
{
	unique_lock<mutex> lock(mPath);
	return refine_path;
}

/**
 * @brief 返回位图可视化
 * @return
 */
CameraPoseVisualization* KeyFrameDatabase::getPosegraphVisualization()
{
	unique_lock<mutex> lock(mPosegraphVisualization);
	return posegraph_visualization;
}