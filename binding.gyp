{
  'targets': [{
    'target_name': 'kinect',
    'sources': ['src/kinect.cc'],
    'libraries': ['-lfreenect'],
    'cflags': ['-std=c++11']
  }]
}

