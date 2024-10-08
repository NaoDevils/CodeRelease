target_sources(${PROJECT_NAME}
    PRIVATE
        Arms.h
        AudioBuffer.h
        AudioBuffer.cpp
        Boundary.h
        Build.h
        Cabsl.h
        ColorModelConversions.h
        ColorRGBA.cpp
        ColorRGBA.h
        Configuration/RobotConfig.h
        Debugging/Annotation.h
        Debugging/AnnotationManager.cpp
        Debugging/AnnotationManager.h
        Debugging/CSVLogger.cpp
        Debugging/CSVLogger.h
        Debugging/DebugDataStreamer.cpp
        Debugging/DebugDataStreamer.h
        Debugging/DebugDataTable.cpp
        Debugging/DebugDataTable.h
        Debugging/DebugDrawings.cpp
        Debugging/DebugDrawings.h
        Debugging/DebugDrawings3D.cpp
        Debugging/DebugDrawings3D.h
        Debugging/DebugImages.h
        Debugging/DebugRequest.cpp
        Debugging/DebugRequest.h
        Debugging/Debugging.h
        Debugging/Modify.h
        Debugging/QueueFillRequest.cpp
        Debugging/QueueFillRequest.h
        Debugging/Stopwatch.h
        Debugging/TcpConnection.cpp
        Debugging/TcpConnection.h
        Debugging/TimingManager.cpp
        Debugging/TimingManager.h
        Debugging/Watch.h
        DynamicRingBuffer.h
        Enum.h
        Global.cpp
        Global.h
        ImageProcessing/Vector2D.h
        ImageProcessing/stb_image.h
        ImageProcessing/stb_image_write.h
        Joints.h
        Limbs.h
        Math/Angle.h
        Math/Approx.h
        Math/BHMath.h
        Math/Bspline.h
        Math/Constants.h
        Math/Covariance.h
        Math/Eigen.h
        Math/EigenArrayExtensions.h
        Math/EigenMatrixBaseExtensions.h
        Math/FifthOrderPolynomial.cpp
        Math/FifthOrderPolynomial.h
        Math/Filter/FIRFilter.cpp
        Math/Filter/FIRFilter.h
        Math/Filter/FastFilter.h
        Math/Filter/Filter.cpp
        Math/Filter/Filter.h
        Math/Fourier.cpp
        Math/Fourier.h
        Math/GaussianDistribution.cpp
        Math/GaussianDistribution.h
        Math/GaussianDistribution2D.cpp
        Math/GaussianDistribution2D.h
        Math/GaussianDistribution3D.cpp
        Math/GaussianDistribution3D.h
        Math/Geometry.cpp
        Math/Geometry.h
        Math/Kalman.h
        Math/Kalman/KalmanMultiDimensional.h
        Math/Kalman/KalmanPositionTracking2D.h
        Math/PolynomialSolver.cpp
        Math/PolynomialSolver.h
        Math/Pose2f.h
        Math/Pose3f.h
        Math/Probabilistics.h
        Math/RadialBasisFunctionApproximator.cpp
        Math/RadialBasisFunctionApproximator.h
        Math/Random.cpp
        Math/Random.h
        Math/Rotation.h
        Math/RotationMatrix.cpp
        Math/RotationMatrix.h
        Math/Transformation.cpp
        Math/Transformation.h
        Math/sse_mathfun.h
        MessageQueue/InMessage.cpp
        MessageQueue/InMessage.h
        MessageQueue/LogFileFormat.h
        MessageQueue/MessageIDs.h
        MessageQueue/MessageQueue.cpp
        MessageQueue/MessageQueue.h
        MessageQueue/MessageQueueBase.cpp
        MessageQueue/MessageQueueBase.h
        MessageQueue/OutMessage.cpp
        MessageQueue/OutMessage.h
        Modeling/BallPhysics.h
        Modeling/Obstacle.cpp
        Modeling/Obstacle.h
        Modeling/PT2.cpp
        Modeling/PT2.h
        Modeling/PoseComputation.cpp
        Modeling/PoseComputation.h
        Modeling/PoseGenerator.h
        Module/Blackboard.cpp
        Module/Blackboard.h
        Module/Logger.cpp
        Module/Logger.h
        Module/Module.cpp
        Module/Module.h
        Module/ModuleManager.cpp
        Module/ModuleManager.h
        Module/ModulePackage.h
        Module/Next.h
        Motion/CanonicalSystem.cpp
        Motion/CanonicalSystem.h
        Motion/DynamicMotionPrimitive.h
        Motion/ForwardKinematic.cpp
        Motion/ForwardKinematic.h
        Motion/InverseKinematic.cpp
        Motion/InverseKinematic.h
        Motion/ZmpPreviewController3.cpp
        Motion/ZmpPreviewController3.h
        Network/TcpComm.cpp
        Network/TcpComm.h
        Network/UdpComm.cpp
        Network/UdpComm.h
        Optimization/FunctionMinimizer.cpp
        Optimization/FunctionMinimizer.h
        Optimization/GaussNewtonOptimizer.h
        Optimization/ParticleSwarm.cpp
        Optimization/ParticleSwarm.h
        PIDController.h
        PerceptsPerSecond.h
        PotentialField.cpp
        PotentialField.h
        ProcessFramework/CycleLocal.h
        ProcessFramework/ExecutorObserver.cpp
        ProcessFramework/ExecutorObserver.h
        ProcessFramework/PlatformProcess.h
        ProcessFramework/Process.cpp
        ProcessFramework/Process.h
        ProcessFramework/ProcessFramework.cpp
        ProcessFramework/ProcessFramework.h
        ProcessFramework/Receiver.cpp
        ProcessFramework/Receiver.h
        ProcessFramework/Sender.cpp
        ProcessFramework/Sender.h
        ProcessFramework/SubThread.cpp
        ProcessFramework/SubThread.h
        Protobuf/ProtobufTools.cpp
        Protobuf/ProtobufTools.h
        Range.h
        RingBuffer.h
        RingBufferWithSum.h
        RobotParts/FootShape.cpp
        RobotParts/FootShape.h
        SSE.h
        SIMD.h
        SampleSet.h
        SensorData.h
        Settings.cpp
        Settings.h
        SortingNetworks.h
        Streams/AutoStreamable.h
        Streams/Compressed.cpp
        Streams/Compressed.h
        Streams/Eigen.h
        Streams/InOut.cpp
        Streams/InOut.h
        Streams/InStreams.cpp
        Streams/InStreams.h
        Streams/OutStreams.cpp
        Streams/OutStreams.h
        Streams/RobotParameters.cpp
        Streams/RobotParameters.h
        Streams/SimpleMap.cpp
        Streams/SimpleMap.h
        Streams/StreamHandler.cpp
        Streams/StreamHandler.h
        Streams/Streamable.cpp
        Streams/Streamable.h
        TripleBuffer.h
        date.h
 )
