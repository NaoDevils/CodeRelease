// generic named struct to keep any color, RGB, HSL, etc.
struct ABC
{
  int a, b, c;
};

class KMeans
{
public:
  KMeans(const std::vector<ABC>& data, int maxIter, int nClasses) : data(data), maxIter(maxIter), nClasses(nClasses) { initialize(); }

  void run()
  {
    for (int i = 0; i < maxIter; i++)
    {
      assignClusters();
      updateCentroids();
    }
  }

  const std::vector<int>& getLabels() const { return labels; }

  const std::vector<ABC>& getCentroids() const { return centroids; }
  const std::vector<int>& getCounts() const { return count; }

  std::vector<int> predict(const std::vector<ABC>& newData) const
  {
    std::vector<int> newLabels(newData.size());
    for (size_t i = 0; i < newData.size(); i++)
    {
      newLabels[i] = closestCentroid(newData[i]);
    }
    return newLabels;
  }

private:
  std::vector<ABC> data;
  int maxIter;
  int nClasses;
  std::vector<ABC> centroids;
  std::vector<int> count;
  std::vector<int> labels;

  void initialize()
  {
    centroids.resize(nClasses);
    count.resize(nClasses);
    for (int i = 0; i < nClasses; i++)
    {
      centroids[i] = data[std::rand() % data.size()];
      count[i] = 0;
    }
  }

  void assignClusters()
  {
    labels.resize(data.size());
    for (size_t i = 0; i < data.size(); i++)
    {
      labels[i] = closestCentroid(data[i]);
    }
  }

  void updateCentroids()
  {
    std::vector<ABC> newCentroids(nClasses, {0, 0, 0});
    for (int i = 0; i < nClasses; i++)
      count[i] = 0;

    for (size_t i = 0; i < data.size(); i++)
    {
      int cluster = labels[i];
      newCentroids[cluster].a += data[i].a;
      newCentroids[cluster].b += data[i].b;
      newCentroids[cluster].c += data[i].c;
      ++count[cluster];
    }

    for (int i = 0; i < nClasses; i++)
    {
      if (count[i] != 0)
      {
        centroids[i].a = newCentroids[i].a / count[i];
        centroids[i].b = newCentroids[i].b / count[i];
        centroids[i].c = newCentroids[i].c / count[i];
      }
    }
  }

  int closestCentroid(const ABC& point) const
  {
    int closest = 0;
    double minDist = std::numeric_limits<double>::max();

    for (int i = 0; i < nClasses; ++i)
    {
      double dist = distance(point, centroids[i]);
      if (dist < minDist)
      {
        minDist = dist;
        closest = i;
      }
    }

    return closest;
  }

  double distance(const ABC& col1, const ABC& col2) const { return std::pow(col1.a - col2.a, 2) + std::pow(col1.b - col2.b, 2) + std::pow(col1.c - col2.c, 2); }
};
