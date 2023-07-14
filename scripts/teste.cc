#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace gazebo {
	class WorldPluginTutorial : public WorldPlugin {
		
		//Declaração das variáveis utilizadas
		private:ros::NodeHandle *rosNode;
		
		private:float x_prime = 0.0;
		private:float y_prime = 0.0;
		private:float theta_prime = 0.0;
		
		private:float current_x = 0.0;
		private:float current_y = 0.0;
		private:float current_z = 0.0;
		private:float current_theta = 0.0;
		
		private:double roll, pitch, yaw;
		private:geometry_msgs::Pose newPose;
		private:ros::ServiceClient Setclient;
		private:ros::ServiceClient Getclient;
		ros::Timer timer;
		
		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
			//Inicialização do plugin e do nó usado para a publicação
			ros::NodeHandle nh;
			int argc = 0;
			char** argv = NULL;
			ros::init(argc,argv,"teste_plugin", ros::init_options::NoSigintHandler);
			this->rosNode = new ros::NodeHandle("teste_plugin");
			
			//Criação das funções utilizadas para obtenção da posição do robô, para a publicação da posição pós sequestro, e timer do callback
			this->Setclient = this->rosNode->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
			this->Getclient = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
			this->timer = this->rosNode->createTimer(ros::Duration(1.0), &WorldPluginTutorial::timerCallback, this);
			//Existem outras formas mais simples de se fazer isso, mas se não for dessa forma uma vez que a função load acabar essas 3 funções deixam de existir e param de ser executadas, isso é sensível especialmente para o timer utilizado.
			
		}
	
         	//Função principal do Sequestro, que será chamada periodicamente de acordo com o timer	
		void timerCallback(const ros::TimerEvent& event){
			//Aquisição da posição atual (x,y,theta)
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name = "husky";
			this->Getclient.call(getmodelstate);
			current_x = getmodelstate.response.pose.position.x;
			current_y = getmodelstate.response.pose.position.y;
			current_z = getmodelstate.response.pose.position.z;
			tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z, getmodelstate.response.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);
			current_theta = yaw;
					
			//Cálculo dos Parâmetros do sequestro
			//Esses valores podem ser alterados diretamente, ou podem ser criadas variáveis para que a manipulação desses parâmetros sejam mais simples
			//Ex: x_prime = current_x + seq_x; 
			x_prime = current_x + 1.0;
			y_prime = current_y + 0.5;
			theta_prime = current_theta + 1.57079633;		
					
			//Armazenamento dos parâmetros do sequestro no objeto newPose 
			newPose.position.x = x_prime;
			newPose.position.y = y_prime;
			
			//Unroll theta_prime into a quaternion
			tf::Quaternion quat;
			quat.setRPY(0, 0, theta_prime);//ROLL, PITCH *YAW*
			newPose.orientation.x = quat.x();
			newPose.orientation.y = quat.y();
		 	newPose.orientation.z = quat.z();
			newPose.orientation.w = quat.w();
		
			//Publicação do newPose para o modelstate do gazebo
			gazebo_msgs::ModelState modelstate;
			modelstate.model_name = (std::string) "husky";
			modelstate.reference_frame = (std::string) "world";
			modelstate.pose = newPose;
			gazebo_msgs::SetModelState setmodelstate;
			setmodelstate.request.model_state = modelstate;
			this->Setclient.call(setmodelstate);
		
			//Print dos dados no terminal
			//Nessa seção pode ser feita outra publicação para que os dados possam ser registrados com rosbags
			printf("Moved the robot from %f, %f to %f, %f \n", current_x,current_y, x_prime, y_prime);
			printf("Changed theta from %f to %f \n\n", current_theta, theta_prime);
			
		}//Final do timerCallback
	
	};//Final da classe
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

}//Final do namespace
