gcc -lmmal_core -lmmal_util -lmmal_vc_client -lvcos -lbcm_host -lbrcmGLESv2 -lbrcmEGL -lm  -pthread -I./ -I./gl_scenes -I/opt/vc/include/ -I/opt/vc/include/interface/vctypes/ -I/opt/vc/include/interface/vcsm -L/opt/vc/lib/ -L./ -o mipi327cap  main.c mipi327_cam_stream.c  RaspiCamControl.c RaspiCLI.c RaspiPreview.c libveyecam.a
