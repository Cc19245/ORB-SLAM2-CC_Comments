/**
 * @file LocalMapping.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 局部建图线程
 * @version 0.1
 * @date 2019-04-29
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

// 构造函数
LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
    /*
     * mbStopRequested：    外部线程调用，为true，表示外部线程请求停止 local mapping
     * mbStopped：          为true表示可以并终止localmapping 线程
     * mbNotStop：          true，表示不要停止 localmapping 线程，因为要插入关键帧了。需要和 mbStopped 结合使用
     * mbAcceptKeyFrames：  true，允许接受关键帧。tracking 和local mapping 之间的关键帧调度
     * mbAbortBA：          是否流产BA优化的标志位
     * mbFinishRequested：  请求终止当前线程的标志。注意只是请求，不一定终止。终止要看 mbFinished
     * mbResetRequested：   请求当前线程复位的标志。true，表示一直请求复位，但复位还未完成；表示复位完成为false
     * mbFinished：         判断最终LocalMapping::Run() 是否完成的标志。
     */
}

// 设置回环检测线程句柄
void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

// 设置追踪线程句柄
void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

// 线程主函数
void LocalMapping::Run()
{

    // 标记状态，表示当前run函数正在运行，尚未结束
    mbFinished = false;
    // 主循环
    while(1)
    {
        // Tracking will see that Local Mapping is busy
        // Step 1 告诉Tracking，LocalMapping正处于繁忙状态，请不要给我发送关键帧打扰我
        // LocalMapping线程处理的关键帧都是Tracking线程发来的
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        // 等待处理的关键帧列表不为空
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            // Step 2 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
            //; 计算这个关键帧的词袋向量，对关键帧中的地图点添加这个关键帧对地图点的观测关系，然后更新这个地图点的观测方向、平均深度等信息
            //; 最后把这个关键帧加入到共视图中，更新这个关键帧的共视图链接关系（相当于初始化这个关键帧的自己的共视关系）
            ProcessNewKeyFrame();

            //; 下面Step 3 和 4的顺序有点奇怪，因为3中剔除的点实际是4中生成的新的地图点，也就是本次生成的点会在下一次再被剔除
            //; 
            // Check recent MapPoints
            // Step 3 根据地图点的观测情况剔除质量不好的地图点
            MapPointCulling();

            // Triangulate new MapPoints
            // Step 4 当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
            //; 通过词典匹配当前帧和其共视关键帧之间的特征点，使用极线约束来剔除外点（注意极线约束是作为筛选条件的，而不是用来搜索的）
            //; 对匹配后的点进行双向投影得到误差，进一步进行筛选
            CreateNewMapPoints();  // 目前来看，是当前帧生成的地图点，到下一帧在生成地图点的时候再culling，那最后那一帧生成的地图点
            // 不就不会执行culling了吗？只有下一次调用局部见图线程的时候才会执行啊

            // 已经处理完队列中的最后的一个关键帧
            //; 这里全部处理完队列中新插入的关键帧才去融合当前关键帧和相邻关键帧的地图点，
            //; 那么中间的那些新插入的关键帧地图点不就是没有被融合吗？
            if(!CheckNewKeyFrames())  
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                //  Step 5 检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                SearchInNeighbors();
            }

            // 终止BA的标志
            mbAbortBA = false;

            // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                // Step 6 当局部地图中的关键帧大于2个的时候进行局部地图的BA
                //; 局部地图关键帧个数不能太少
                if(mpMap->KeyFramesInMap()>2)
                    // 注意这里的第二个参数是按地址传递的,当这里的 mbAbortBA 状态发生变化时，能够及时执行/停止BA
                    //; 把当前帧的一级共视关键帧和他们的地图点作为g2o优化的顶点，加入g2o优化，同时优化地图点和位姿。
                    //; 此外，会把当前帧的二级共视关键帧也加入到g2o中，但是不优化这些帧的位姿，只是作为一个约束
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);  // 局部BA

                // Check redundant local Keyframes
                // Step 7 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                // 冗余的判定：该关键帧的90%的地图点可以被其它关键帧观测到
                //; 注意这个函数判定是冗余关键帧之后，然后设置这个关键帧的BadFlag
                KeyFrameCulling();   
            }

            // Step 8 将当前帧加入到闭环检测队列中
            // 注意这里的关键帧被设置成为了bad的情况,这个需要注意
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())     // 当要终止当前线程的时候
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                // 如果还没有结束利索,那么等
                // usleep(3000);
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
            // 然后确定终止了就跳出这个线程的主循环
            if(CheckFinish())
                break;
        }

        // 查看是否有复位线程的请求
        ResetIfRequested();

        // Tracking will see that Local Mapping is not busy
        SetAcceptKeyFrames(true);

        // 如果当前线程已经结束了就跳出主循环
        if(CheckFinish())
            break;          //; 注意这里是跳出最外面while()的循环

        //usleep(3000);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

    }

    // 设置线程已经终止
    SetFinish();
}

// 插入关键帧,由外部（Tracking）线程调用;这里只是插入到列表中,等待线程主函数对其进行处理
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    // 将关键帧插入到列表中
    mlNewKeyFrames.push_back(pKF);   //; 注意这里是插入到等待处理的关键帧列表中
    mbAbortBA=true;
}

// 查看列表中是否有等待被插入的关键帧,
bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

/**
 * @brief 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
 * 
 */
void LocalMapping::ProcessNewKeyFrame()
{
    // Step 1：从缓冲队列中取出一帧关键帧
    // 该关键帧队列是Tracking线程向LocalMapping中插入的关键帧组成
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        // 取出列表中最前面的关键帧，作为当前要处理的关键帧
        //; 成员变量，当前正在处理的关键帧
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        // 取出最前面的关键帧后，在原来的列表里删掉该关键帧
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    // Step 2：计算该关键帧特征点的词袋向量
    //; 这个函数里并不一定会直接计算，而是会判断是否已经有了词袋向量，因为关键帧是从普通帧来的，普通帧中可能计算过词袋
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    // Step 3：当前处理关键帧中有效的地图点，更新normal，描述子等信息
    // TrackLocalMap中和当前帧新匹配上的地图点和当前关键帧进行关联绑定
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    // 对当前处理的这个关键帧中的所有的地图点展开遍历
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                //; 如果这个地图点没有添加被当前关键帧观测到的信息。其实我感觉这里说的是废话，你现在处理的都是新插入的关键帧，也就是前面新
                //; 生成的关键帧，怎么可能添加过对这个地图点的观测关系？
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))  // 如果这个地图点没有被新插入的这个关键帧观测到
                {
                    // 如果地图点不是来自当前帧的观测（比如来自局部地图点），为当前地图点添加观测
                    pMP->AddObservation(mpCurrentKeyFrame, i);   // i是这个地图点在关键帧中的2D特征点索引
                    //; 因为当前这个新的关键帧看到了这个地图点，所以要更新这个地图点的平均观测方向和观测距离范围
                    pMP->UpdateNormalAndDepth();
                    //; 同理，因为有新的关键帧看到了这个地图点，所以要更新地图点的最佳描述子
                    pMP->ComputeDistinctiveDescriptors();
                }
                //; 注意看下面的注释，原作者说了这这种情况只在双目的时候发生，所以上面的那个判断就是为了区分单双目
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 如果当前帧中已经包含了这个地图点,但是这个地图点中却没有包含这个关键帧的信息
                    // 这些地图点可能来自双目或RGBD跟踪过程中新生成的地图点，或者是CreateNewMapPoints 中通过三角化产生
                    // 将上述地图点放入mlpRecentAddedMapPoints，等待后续MapPointCulling函数的检验
                    mlpRecentAddedMapPoints.push_back(pMP); 
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    // Step 4：更新关键帧间的连接关系（共视图）
    //; 因为新增加了一个关键帧，就需要建立它和已有的关键帧的链接关系，方法很简单：
    //; 就是遍历这个关键帧的所有地图点，查找也观测到这个地图点的其他关键帧，最后统计共同观测的地图点的数目，就可以得到共视图的权重。
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    // Step 5：将该关键帧插入到地图中
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

/**
 * @brief 检查新增地图点，根据地图点的观测情况剔除质量不好的新增的地图点
 * mlpRecentAddedMapPoints：存储新增的地图点，这里是要删除其中不靠谱的
 */
void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;  //; 最新插入的关键帧ID

    // Step 1：根据相机类型设置不同的观测阈值
    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;
	
	// Step 2：遍历检查新添加的地图点
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            // Step 2.1：已经是坏点的地图点仅从队列中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        //; 这个地图点  被普通帧看到并且匹配上的次数 / 在普通帧的视野里的次数  <  0.25f
        else if(pMP->GetFoundRatio()<0.25f)
        {
            // Step 2.2：跟踪到该地图点的帧数相比预计可观测到该地图点的帧数的比例小于25%，从地图中删除
            // (mnFound/mnVisible） < 25%
            // mnFound ：地图点被多少帧（包括普通帧）看到，次数越多越好
            // mnVisible：地图点应该被看到的次数  //; ???什么意思
            // (mnFound/mnVisible）：对于大FOV镜头这个比例会高，对于窄FOV镜头这个比例会低
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        // mnFirstKFid是在生成地图点的时候的关键帧的Id
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // Step 2.3：从该点建立开始，到现在已经过了不小于2个关键帧
            // 但是观测到该点的关键帧却不超过阈值cnThObs，从地图中删除
            //; 注意这里的地图点是两个共视关键帧三角化恢复出来的，所以恢复完之后它的最初的Obs关键帧观测次数就是2，也就是cnThObs，
            //; 然后这里判断如果从最初生成这个地图点到现在，如果已经过去大于两个关键帧了，没有任何新的一帧看到这个地图点，
            //; 说明恢复出来的这个地图点特性不太好
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            // Step 2.4：从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
            // 因此没有SetBadFlag()，仅从队列中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

/**
 * @brief 用当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
 * 
 */
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    // nn表示搜索最佳共视关键帧的数目
    // 不同传感器下要求不一样,单目的时候需要有更多的具有较好共视关系的关键帧来建立地图
    int nn = 10;
    if(mbMonocular)
        nn=20;

    // Step 1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    // 特征点匹配配置 最佳距离 < 0.6*次佳距离，比较苛刻了。不检查旋转
    ORBmatcher matcher(0.6,false); 

    // 取出当前帧从世界坐标系到相机坐标系的变换矩阵
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));

    // 得到当前关键帧（左目）光心在世界坐标系中的坐标、内参
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    // 用于后面的点深度的验证;这里的1.5是经验值
    // mfScaleFactor = 1.2
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    // 记录三角化成功的地图点数目
    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    // Step 2：遍历相邻关键帧，搜索匹配并用极线约束剔除误匹配，最终三角化
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        // ! 疑似bug，正确应该是 if(i>0 && !CheckNewKeyFrames())
        //; 确实，不知道这里在判断什么？i>0是什么条件？
        //; 如果还有新插入的关键帧没有处理，那么就别恢复地图点了，赶紧去处理下一帧？
        //; 没看懂？？？这到底在判断什么？？考虑因素是什么？
        if(i>0 && CheckNewKeyFrames())  
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        // 相邻的关键帧光心在世界坐标系中的坐标
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        // 基线向量，两个关键帧间的相机位移
        cv::Mat vBaseline = Ow2-Ow1;
        // 基线长度
        const float baseline = cv::norm(vBaseline);

        // Step 3：判断相机运动的基线是不是足够长
        if(!mbMonocular)
        {
            // 如果是双目相机，关键帧间距小于本身的基线时不生成3D点
            // 因为太短的基线下能够恢复的地图点不稳定
            if(baseline<pKF2->mb)
            continue;
        }
        else    
        {
            // 单目相机情况
            // 相邻关键帧的场景深度中值
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            // 基线与景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            // 如果比例特别小，基线太短恢复3D点不准，那么跳过当前邻接的关键帧，不生成3D点
            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        // Step 4：根据两个关键帧的位姿计算它们之间的基础矩阵
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        // Step 5：通过词袋对两关键帧的未匹配的特征点快速匹配，用极线约束抑制离群点，生成新的匹配点对
        //; 注意这里，还是使用词袋进行特征点的匹配，而极线约束只是被用于筛选外点。如果使用极线约束进行匹配，也就是极线搜索的话，
        //; 那这样匹配的工作量太大，速度就太慢了
        vector<pair<size_t,size_t> > vMatchedIndices;
        //; 同时注意这里进行匹配的是两个帧中都没有对应的地图点的哪些特征点，只要其中一帧的特征点有地图点，那么就不匹配它了。
        //; 我感觉这里有道理也没有道理: 
        //; 1.说他有道理是因为这个函数寻找匹配是为了后面三角化生成新的地图点，所以如果某一帧已经有了匹配的
        //;   地图点了的话，那么后面就不能再用它的匹配三角化生成新的地图点了。
        //; 2.说他没道理的话是因为这里实际上漏了一些匹配关系，因为追踪得到的当前关键帧的地图点如果是匀速跟踪的话都是从上一帧
        //;   的地图点中得到的，属于当前帧和上一帧的共视部分。那么很可能有一部分点是当前帧和当前帧的共视关键帧可以看到的，但是却
        //;   不是上一帧能看到的，所以当前帧天然的就和这部分共视关键帧的某些地图点有匹配关系，也就是地图点的观测关系，但是
        //;   但是这个关系却没有被添加上。估计是因为这个工作量太大了吧
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        // Step 6：对每对匹配通过三角化生成3D点,和 Triangulate函数差不多
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // Step 6.1：取出匹配特征点

            // 当前匹配对在当前关键帧中的索引
            const int &idx1 = vMatchedIndices[ikp].first;
            
            // 当前匹配对在邻接关键帧中的索引
            const int &idx2 = vMatchedIndices[ikp].second;

            // 当前匹配在当前关键帧中的特征点
            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;  //; 单目是false

            // 当前匹配在邻接关键帧中的特征点
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;  //; 单目是false

            // Check parallax between rays
            // Step 6.2：利用匹配点反投影得到视差角
            // 特征点反投影,其实得到的是在各自相机坐标系下的一个非归一化的方向向量,和这个点的反投影射线重合
            //; 因为此时有相机的位姿，所以可以把得到这个特征点的方向向量
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            // 由相机坐标系转到世界坐标系(得到的是那条反投影射线的一个同向向量在世界坐标系下的表示,还是只能够表示方向)，得到视差角余弦值
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            // 匹配点射线夹角余弦值
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            //; 下面这个三个变量对于单目来说都没有用
            // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
            float cosParallaxStereo = cosParallaxRays+1;  
            // cosParallaxStereo1、cosParallaxStereo2在后面可能不存在，需要初始化为较大的值
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            // Step 6.3：对于双目，利用双目得到视差角；单目相机没有特殊操作
            if(bStereo1)
                // 传感器是双目相机,并且当前的关键帧的这个点有对应的深度
                // 假设是平行的双目相机，计算出双目相机观察这个点的时候的视差角余弦
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)  //; 这里用elif也不太对吧？肯定这俩同时是true或者false吧？
                // 传感器是双目相机,并且邻接的关键帧的这个点有对应的深度，和上面一样操作
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));
            
            // 得到双目观测的视差角中最小的那个
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            // Step 6.4：三角化恢复3D点
            cv::Mat x3D;
            // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视差角正常,0.9998 对应1°
            // cosParallaxRays < cosParallaxStereo 表明匹配点对夹角大于双目本身观察三维点夹角
            // 匹配点对夹角大，用三角法恢复3D点
            // 参考：https://github.com/raulmur/ORB_SLAM2/issues/345
            //; 对于单目来说，有效的判断就是最后一个，也就是相机光心指向3D点的向量之间的夹角>1度。  ？？？ 1度是不是太小了？？
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                // 见Initializer.cc的 Triangulate 函数,实现是一样的,顶多就是把投影矩阵换成了变换矩阵
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();
                // 归一化之前的检查
                if(x3D.at<float>(3)==0)
                    continue;
                // 归一化成为齐次坐标,然后提取前面三个维度作为欧式坐标
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
            }
            // 匹配点对夹角小，用双目恢复3D点
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)  
            {
                // 如果是双目，用视差角更大的那个双目信息来恢复，直接用已知3D点反投影了
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)  
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            //; 对于单目来说，如果两个向量的角度太小了，那么认为恢复出来的3D点的坐标不准确，直接放弃这个点
            else
                continue; //No stereo and very low parallax, 放弃

            // 为方便后续计算，转换成为了行向量
            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            // Step 6.5：检测生成的3D点是否在相机前方,不在的话就放弃这个点
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            // Step 6.6：计算3D点在当前关键帧下的重投影误差
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            //; 单目情况
            if(!bStereo1)
            {
                // 单目情况下
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                // 双目情况
                float u1 = fx1*x1*invz1+cx1;
                // 根据视差公式计算假想的右目坐标
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;     
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                // 自由度为3，卡方检验阈值是7.8
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            // 计算3D点在另一个关键帧下的重投影误差，操作同上
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            // Step 6.7：检查尺度连续性

            // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            // ratioDist是不考虑金字塔尺度下的距离比例
            //; 距离越远，那么金字塔层数越低。假设dist2更大，那么他所在的金字塔缩放系数就小。假设dist2/dist1 = 1.2/1, 
            //; 那么层数缩放系数比应该大致满足 1/1.2
            const float ratioDist = dist2/dist1;
            // 金字塔尺度因子的比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/

            // 距离的比例和图像金字塔的比例不应该差太多，否则就跳过
            //; ratioFactor = 1.5 * factor = 1.5 * 1.2
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            // Step 6.8：三角化生成3D点成功，构造成MapPoint
            //; 注意地图点的RedKF就是生成它的那个关键帧，也就是处理的当前关键帧
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            // Step 6.9：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            //; 从这里可以看出来，新生成的这些地图点只是被关键帧观测到，目前还没有被普通帧观测到
            //; 注意：nObs是只有关键帧才能修改的，这个可以翻译为观测。其他的要翻译成找到（found）和可见（visible）
            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            // b.该MapPoint的描述子
            pMP->ComputeDistinctiveDescriptors();

            // c.该MapPoint的平均观测方向和深度范围
            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);

            // Step 6.10：将新产生的点放入检测队列
            // 这些MapPoints都会经过MapPointCulling函数的检验
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

/**
 * @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的地图点
 * 
 */
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    // Step 1：获得当前关键帧在共视图中权重排名前nn的邻接关键帧
    // 开始之前先定义几个概念
    // 当前关键帧的邻接关键帧，称为一级相邻关键帧，也就是邻居
    // 与一级相邻关键帧相邻的关键帧，称为二级相邻关键帧，也就是邻居的邻居

    // 单目情况要20个邻接关键帧，双目或者RGBD则要10个
    int nn = 10;
    if(mbMonocular)
        nn=20;

    // 和当前关键帧相邻的关键帧，也就是一级相邻关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    
    // Step 2：存储一级相邻关键帧及其二级相邻关键帧
    vector<KeyFrame*> vpTargetKFs;
    // 开始对所有候选的一级关键帧展开遍历：
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // 没有和当前帧进行过融合的操作
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        // 加入一级相邻关键帧    
        vpTargetKFs.push_back(pKFi);
        // 标记已经加入
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        // 以一级相邻关键帧的共视关系最好的5个相邻关键帧 作为二级相邻关键帧
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        // 遍历得到的二级相邻关键帧
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            // 当然这个二级相邻关键帧要求没有和当前关键帧发生融合,并且这个二级相邻关键帧也不是当前关键帧
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            // 存入二级相邻关键帧    
            vpTargetKFs.push_back(pKFi2);
        }
    }

    // Search matches by projection from current KF in target KFs
    // 使用默认参数, 最优和次优比例0.6,匹配时检查特征点的旋转
    ORBmatcher matcher;

    // Step 3：将当前帧的地图点分别投影到两级相邻关键帧，寻找匹配点对应的地图点进行融合，称为正向投影融合
    //; 这里的融合是有道理的。因为如果没有局部建图这种操作的话，那么所有帧的地图点都属于初始化的地图点的一部分，所以就不存在融合的说法。
    //; 但是因为实际上局部建图中最重要的一个操作就是三角化产生新的地图点，所以新生成的地图点可能会和之前关键帧中有的地图点很相近。
    //; 问题：这里是把所有的地图点都进行融合了，只融合上面三角化新生成的地图点不行吗？
    //; 按照目前作者的这种写法是不行的，我感觉总体的原因是作者的这个写法有bug？还是说作者出于效率考虑？
    //; 因为当前帧新三角化的地图点肯定要融合，当前帧跟踪时得到的地图点是来自上一帧的（匀速模型跟踪的话），那上一帧的地图点又来自哪里？
    //; 上一帧的地图点来自上上帧......再往前追溯一定可以找到最近的那个关键帧，所以当前帧的地图点有一部分是属于上一帧的地图点的，此外
    //; 还有一部分是在TrackLocalMap的时候当前帧的共视关键帧的。按照作者当前的做法，所有新插入到LocalMapping中的关键帧，是处理完毕
    //; 后才进行一次地图融合，所以当前帧的这些共视关键帧新生成的地图点可能没有经过融合，那么这些点中又有一部分作为当前帧追踪时的地图点,
    //; 所以这部分地图点也是需要融合的。所以说感觉作者的这种写法有些混乱，可能是为了效率考虑？因为实际上不论你怎么融合，都会有遗漏的地图点
    //; 没有融合到，除非你遍历所有的关键帧。

    //; 我认为如果不存在效率问题的话，对插入到每个LocalMappping中的关键帧，在三角化生成地图点之后，接着就进行一次对新生成的地图点
    //; 的融合。
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        // 将地图点投影到关键帧中进行匹配和融合；融合策略如下
        // 1.如果地图点能匹配关键帧的特征点，并且该点有对应的地图点，那么选择观测数目多的替换两个地图点
        // 2.如果地图点能匹配关键帧的特征点，并且该点没有对应的地图点，那么为该点添加该投影地图点
        // 注意这个时候对地图点融合的操作是立即生效的
        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    // Step 4：将两级相邻关键帧地图点分别投影到当前关键帧，寻找匹配点对应的地图点进行融合，称为反向投影融合
    //; 离谱，这得有多少地图点啊！
    // 用于进行存储要融合的一级邻接和二级邻接关键帧所有MapPoints的集合
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());
    
    //  Step 4.1：遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpFuseCandidates
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;
        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            
            // 如果地图点是坏点，或者已经加进集合vpFuseCandidates，跳过
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;

            // 加入集合，并标记已经加入
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }
    // Step 4.2：进行地图点投影融合,和正向融合操作是完全相同的
    // 不同的是正向操作是"每个关键帧和当前关键帧的地图点进行融合",而这里的是"当前关键帧和所有邻接关键帧的地图点进行融合"
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);

    // Update points
    // Step 5：更新当前帧地图点的描述子、深度、平均观测方向等属性
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 在所有找到pMP的关键帧中，获得最佳的描述子
                pMP->ComputeDistinctiveDescriptors();

                // 更新平均观测方向和观测距离
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    // Step 6：更新当前帧与其它帧的共视连接关系
    mpCurrentKeyFrame->UpdateConnections();
}

// 根据两关键帧的姿态计算两个关键帧之间的基本矩阵
cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    // 先构造两帧之间的R12,t12
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    // 得到 t12 的反对称矩阵
    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Essential Matrix: t12叉乘R12
    // Fundamental Matrix: inv(K1)*E*inv(K2)
    return K1.t().inv()*t12x*R12*K2.inv();
}

// 外部线程调用,请求停止当前线程的工作; 其实是回环检测线程调用,来避免在进行全局优化的过程中局部建图线程添加新的关键帧
void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

// 检查是否要把当前的局部建图线程停止工作,运行的时候要检查是否有终止请求,如果有就执行. 由run函数调用
bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    // 如果当前线程还没有准备停止,但是已经有终止请求了,那么就准备停止当前线程
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

// 检查mbStopped是否为true，为true表示可以并终止localmapping 线程
bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

// 求外部线程调用，为true，表示外部线程请求停止 local mapping
bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

// 释放当前还在缓冲区中的关键帧指针
void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

// 查看当前是否允许接受关键帧
bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

// 设置"允许接受关键帧"的状态标志
void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

// 设置 mbnotStop标志的状态
bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    //已经处于!flag的状态了
    // 就是我希望线程先不要停止,但是经过检查这个时候线程已经停止了...
    if(flag && mbStopped)
        //设置失败
        return false;

    //设置为要设置的状态
    mbNotStop = flag;
    //设置成功
    return true;
}

// 终止BA
void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

/**
 * @brief 检测当前关键帧在共视图中的关键帧，根据地图点在共视图中的冗余程度剔除该共视关键帧
 * 冗余关键帧的判定：90%以上的地图点能被其他关键帧（至少3个）观测到
 */
void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    // 该函数里变量层层深入，这里列一下：
    // mpCurrentKeyFrame：当前关键帧，本程序就是判断它是否需要删除
    // pKF： mpCurrentKeyFrame的某一个共视关键帧
    // vpMapPoints：pKF对应的所有地图点
    // pMP：vpMapPoints中的某个地图点
    // observations：所有能观测到pMP的关键帧
    // pKFi：observations中的某个关键帧
    // scaleLeveli：pKFi的金字塔尺度
    // scaleLevel：pKF的金字塔尺度

    // Step 1：根据共视图提取当前关键帧的所有共视关键帧
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 对所有的共视关键帧进行遍历
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        // 第1个关键帧不能删除，跳过
        if(pKF->mnId==0)
            continue;
        // Step 2：提取每个共视关键帧的地图点
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        // 记录某个点被观测次数，后面并未使用
        int nObs = 3;                     
        // 观测次数阈值，默认为3
        const int thObs=nObs;               
        // 记录冗余观测点的数目
        int nRedundantObservations=0;     
                                                                                      
        int nMPs=0;            

        // Step 3：遍历该共视关键帧的所有地图点，其中能被其它至少3个关键帧观测到的地图点为冗余地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        // 对于双目或RGB-D，仅考虑近处（不超过基线的40倍 ）的地图点
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    // pMP->Observations() 是观测到该地图点的相机总数目（单目1，双目2）
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        // Observation存储的是可以看到该地图点的所有关键帧的集合
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();

                        int nObs=0;
                        // 遍历观测到该地图点的关键帧
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            // 尺度约束：为什么pKF 尺度+1 要大于等于 pKFi 尺度？
                            // 回答：因为同样或更低金字塔层级的地图点更准确
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                // 已经找到3个满足条件的关键帧，就停止不找了
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        // 地图点至少被3个关键帧观测到，就记录为冗余点，更新冗余点计数数目
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        // Step 4：如果该关键帧90%以上的有效地图点被判断为冗余的，则认为该关键帧是冗余的，需要删除该关键帧
        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

// 计算三维向量v的反对称矩阵
cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<
            0,              -v.at<float>(2),     v.at<float>(1),
            v.at<float>(2),               0,    -v.at<float>(0),
           -v.at<float>(1),  v.at<float>(0),                 0);
}

// 请求当前线程复位,由外部线程调用,堵塞的
void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    // 一直等到局部建图线程响应之后才可以退出
    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        //usleep(3000);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

    }
}

// 检查是否有复位线程的请求
void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    // 执行复位操作:清空关键帧缓冲区,清空待cull的地图点缓冲
    
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        // 恢复为false表示复位过程完成
        mbResetRequested=false;
    }
}

// 请求终止当前线程
void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

// 检查是否已经有外部线程请求终止当前线程
bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

// 设置当前线程已经真正地结束了
void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    // 线程已经被结束
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;     //既然已经都结束了,那么当前线程也已经停止工作了
}

// 当前线程的run函数是否已经终止
bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
