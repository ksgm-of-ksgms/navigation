module Lib
    ( someFunc
    ) where


import Vis
import Linear.V3

import Control.Concurrent.Async (async, wait)
--import Control.Concurrent.MVar
import Data.IORef
import OpenCV.HighGui
import OpenCV.Core.Types.Mat
import Data.Int (Int32)
import Data.Function (on)
import Control.Applicative

mkSlider :: IORef Int32 -> IO ()
mkSlider mvar = do
    win <- makeWindow "sample"
    resizeWindow win 300 300
    createTrackbar win "x" 0 100 $ \v -> writeIORef mvar v
    imshow win $ eyeMat @Int32  @Int32  @Int32 300 300 3 Depth_8U
    waitKey 0
    destroyWindow win
    return ()

divf = (/) `on` fromIntegral

myrender :: IORef Int32 -> Float -> IO (VisObject Double)
myrender mvar _ = do
    v <- readIORef mvar
    return $ Cube (divf v 100) Solid blue

someFunc :: IO ()
someFunc = do
    mvar <- newIORef @Int32 0
    print "aa"
    tid <- async $ mkSlider mvar
    print "aa"
    animateIO defaultOpts (myrender mvar)
    wait tid
    return ()

