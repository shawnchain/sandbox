package mclub.util.shuffle;

import java.io.File;
import java.io.FileFilter;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.apache.commons.io.FileUtils;

/**
 * Hello world!
 * 
 */
public class ShuffleApp {
	public static void main(String[] args) {
		if(args.length != 2){
			System.out.println("Usage: java -jar shuffle.jar source_folder destination_folder");
			return;
		}
		
		File source = new File(args[0]);
		File destRoot = new File(args[1]);
		
		// make sure both exists
		if(!source.exists() || !source.isDirectory()){
			System.out.println("Source should be a valid folder");
			return;
		}
		if(!destRoot.exists() || !destRoot.isDirectory()){
			System.out.println("Destination should be a valid folder");
			return;
		}
		long start = System.currentTimeMillis();
		
		// walk through the source folder
		System.out.print("Collecting file...");
		LinkedList<File> result = new LinkedList<File>();
		collectFile(source,result);
		System.out.println("" + result.size() + " found");
		
		if(result.size() == 0){
			return;
		}
		
		// shuffle
		System.out.println("Shuffling...");
		LinkedList<File> shuffled = result;
		int sTimes = 1;
		for(int i = 0;i < sTimes;i++){
			// shuffle 10 times
			shuffled =  shuffleFile(shuffled);
		}
		
		
		// copying
		System.out.println("Copying files...");
		int totalFilesCopied = 0;
		int maxFilesPerFolder = 255;
		int folders = shuffled.size() / maxFilesPerFolder + 1;
		File destFolder = null;
		for(int i = 1;i <= folders;i++){
			destFolder = new File(destRoot,"shuffled_" + i);
			destFolder.mkdirs();
			if(!destFolder.exists() || !destFolder.canWrite()){
				System.out.println("Can not write " + destFolder);
				return;
			}
			copyloop:for(;totalFilesCopied < shuffled.size();){
				File f = shuffled.get(totalFilesCopied);
				try{
					FileUtils.copyFileToDirectory(f, destFolder, true);
					totalFilesCopied++;
					if(totalFilesCopied % 10 == 0){
						System.out.println("" + totalFilesCopied + " of " + shuffled.size() + " copied, elapsed " + ((System.currentTimeMillis() - start) / 1000) + " seconds");
					}
					if(totalFilesCopied % maxFilesPerFolder == 0){
						break copyloop;
					}
				}catch(Exception e){
					System.out.println("Error copy " + f + ", " + e.getMessage());
				}
			}
		}
		
		
		System.out.println("Done, elapsed " + ((System.currentTimeMillis() - start) / 1000) + " seconds");
	}
	
	static Random rnd = new Random();
	
	public static LinkedList<File> shuffleFile(LinkedList<File> source){
		LinkedList<File> shuffled = new LinkedList<File>();
		while(source.size() > 0){
			int idx = rnd.nextInt(source.size());
			shuffled.add(source.remove(idx));
		}
		return shuffled;
	}
	
	public static void collectFile(File folder, List<File> result){
		if(!folder.isDirectory()){
			result.add(folder);
			return;
		}
		
		// a directory
		File[] children = folder.listFiles(new FileFilter(){
			public boolean accept(File c) {
				return (c.isDirectory() && !c.getName().equals(".Trashes")) || (c.getName().endsWith(".mp3") && !c.getName().startsWith(".")); 
            }
		});
		
		if(children != null && children.length > 0){
			for(File c : children){
				collectFile(c,result);
			}
		}
	}
}
