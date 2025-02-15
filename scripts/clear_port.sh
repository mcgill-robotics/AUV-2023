processes_on_ports=$(sudo ss -lptn 'sport = :11311')

echo "$processes_on_ports" | sed '1d' | sed -nE 's/.*pid=([0-9]+).*/\1/p' | while read -r pid; do
    sudo kill -9 $pid
done

